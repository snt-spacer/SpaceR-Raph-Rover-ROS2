/**
 * @file bumblebee_node.cpp
 * @brief FLIR Bumblebee X ROS 2 publisher (Spinnaker C++ API).
 *
 * Configures the Bumblebee X stereo camera, disables its raw streams to save
 * network bandwidth (about 18 MB to 6 MB per frame), and publishes the
 * rectified and disparity streams with the depth and point cloud derived from them.
 *
 * Topics published (navcam = forward navigation stereo camera):
 *   navcam/left/image_rect       sensor_msgs/Image  (rgb8)
 *   navcam/right/image_rect      sensor_msgs/Image  (rgb8)
 *   navcam/disparity/image_raw   sensor_msgs/Image  (16UC1)
 *   navcam/depth/image_raw       sensor_msgs/Image  (32FC1, metres)
 *   navcam/points                sensor_msgs/PointCloud2 (organized XYZRGB)
 *   navcam/left/camera_info      sensor_msgs/CameraInfo
 *   navcam/right/camera_info     sensor_msgs/CameraInfo
 *
 * The OpenCV preview window is only shown when the `debug_view` parameter is
 * true (wired to the DEBUG_VIEW env var in docker-compose.yml).
 */

#include <cstring>
#include <string>
#include <memory>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <image_transport/image_transport.hpp>

#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>
#include <ImageUtilityStereo.h>

// OpenCV: depth reconstruction and the optional preview window
#include <opencv2/opencv.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using Img = sensor_msgs::msg::Image;
using CI  = sensor_msgs::msg::CameraInfo;
using PC2 = sensor_msgs::msg::PointCloud2;

/**
 * @struct FrameIntrinsics
 * @brief Stereo intrinsics read from the camera nodemap.
 */
struct FrameIntrinsics {
    double focal_length{0.0};
    double principal_u{0.0};
    double principal_v{0.0};
    double baseline{0.0};
    double coord_scale{1.0};
    double coord_offset{0.0};
    float  invalid_value{0.0f};
    bool   invalid_flag{false};
};

class BumbleBeeNode : public rclcpp::Node
{
public:
    BumbleBeeNode() : Node("bumblebee_publisher")
    {
        // ── Node Parameters ──
        declare_parameter("camera_serial",       "");
        declare_parameter("resolution",          "Quarter");
        declare_parameter("fps",                 2.0);
        declare_parameter("pixel_format",        "RGB8Packed");
        // Reference (left/disparity) optical frame, as defined in the robot URDF.
        declare_parameter("frame_id",            "navcam_optical_frame");
        declare_parameter("enable_speckle_filter", true);
        declare_parameter("max_speckle_size",    1000);
        declare_parameter("speckle_threshold",   3.0);
        declare_parameter("debug_view",          false);

        // ── Runtime-tunable image controls ──
        // Standard GenICam nodes, applied live (rqt_reconfigure / `ros2 param set`).
        // Negative values keep the camera's automatic control.
        declare_parameter("exposure_auto",       true);   // true=Continuous (auto), false=Off (manual)
        declare_parameter("exposure_time",       -1.0);   // microseconds; >0 forces auto off + sets it
        declare_parameter("gain_auto",           true);   // true=Continuous (auto), false=Off (manual)
        declare_parameter("gain",                -1.0);   // dB; >=0 forces auto off + sets it
        declare_parameter("white_balance_auto",  true);   // true=Continuous (auto), false=Off (manual)
        declare_parameter("gamma",               -1.0);   // >=0 to set (enables Gamma)
        declare_parameter("black_level",         -1.0);   // >=0 to set

        serial_          = get_parameter("camera_serial").as_string();
        res_             = get_parameter("resolution").as_string();
        fps_             = get_parameter("fps").as_double();
        fmt_             = get_parameter("pixel_format").as_string();
        speckle_enable_  = get_parameter("enable_speckle_filter").as_bool();
        max_speckle_     = get_parameter("max_speckle_size").as_int();
        speckle_thresh_  = static_cast<float>(get_parameter("speckle_threshold").as_double());
        debug_view_      = get_parameter("debug_view").as_bool();
        frame_id_        = get_parameter("frame_id").as_string();

        // ── ROS 2 Publishers ──
        // image_transport also advertises the compressed variants of each image.
        pub_left_  = image_transport::create_publisher(this, "navcam/left/image_rect");
        pub_right_ = image_transport::create_publisher(this, "navcam/right/image_rect");
        pub_disp_  = image_transport::create_publisher(this, "navcam/disparity/image_raw");
        pub_depth_ = image_transport::create_publisher(this, "navcam/depth/image_raw");
        pub_cloud_ = create_publisher<PC2>("navcam/points",               10);
        pub_linfo_ = create_publisher<CI> ("navcam/left/camera_info",     10);
        pub_rinfo_ = create_publisher<CI> ("navcam/right/camera_info",    10);

        // ── Hardware Initialization ──
        setup_camera();

        // ── Dynamic reconfigure ──
        // Log which controls this specific unit exposes, push any non-default
        // values provided at launch, then register the live callback.
        probe_controls();
        apply_current_controls();
        param_cb_handle_ = add_on_set_parameters_callback(
            std::bind(&BumbleBeeNode::on_params_set, this, std::placeholders::_1));

        // ── Acquisition Timer ──
        double period = 1.0 / fps_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(period),
            std::bind(&BumbleBeeNode::timer_cb, this));

        RCLCPP_INFO(get_logger(),
            "Bumblebee X ready — %.1f Hz  resolution=%s  fmt=%s  speckle_filter=%s  debug_view=%s",
            fps_, res_.c_str(), fmt_.c_str(), speckle_enable_ ? "on" : "off",
            debug_view_ ? "on" : "off");
    }

    ~BumbleBeeNode() override
    {
        if (debug_view_) cv::destroyAllWindows();

        if (cam_) {
            try {
                if (cam_->IsStreaming()) cam_->EndAcquisition();
                cam_->DeInit();
            } catch (...) {}
            cam_ = nullptr;
        }
        cam_list_.Clear();
        if (sys_) sys_->ReleaseInstance();
    }

private:
    // ── GenICam NodeMap Helpers ───────────────────────────────────────────

    static void set_enum(INodeMap& nm, const char* node, const char* entry)
    {
        CEnumerationPtr ptr = nm.GetNode(node);
        if (!IsWritable(ptr)) return;
        CEnumEntryPtr ep = ptr->GetEntryByName(entry);
        if (IsReadable(ep)) ptr->SetIntValue(ep->GetValue());
    }

    static void set_bool(INodeMap& nm, const char* node, bool val)
    {
        CBooleanPtr ptr = nm.GetNode(node);
        if (IsWritable(ptr)) ptr->SetValue(val);
    }

    // Write a float feature, clamping to its advertised range. Returns false if
    // the node is absent/not writable so callers can report it.
    static bool write_float(INodeMap& nm, const char* node, double val)
    {
        CFloatPtr ptr = nm.GetNode(node);
        if (!IsWritable(ptr)) return false;
        const double clamped = std::min(std::max(val, ptr->GetMin()), ptr->GetMax());
        ptr->SetValue(clamped);
        return true;
    }

    static bool node_writable(INodeMap& nm, const char* node)
    {
        CNodePtr ptr = nm.GetNode(node);
        return IsAvailable(ptr) && IsWritable(ptr);
    }

    static double get_float(INodeMap& nm, const char* node)
    {
        CFloatPtr ptr = nm.GetNode(node);
        return IsReadable(ptr) ? ptr->GetValue() : 0.0;
    }

    // ── Camera Configuration ──────────────────────────────────────────────

    void setup_camera()
    {
        RCLCPP_INFO(get_logger(), "Initializing camera (serial=%s)",
            serial_.empty() ? "auto-detect" : serial_.c_str());

        sys_      = System::GetInstance();
        cam_list_ = sys_->GetCameras();

        if (cam_list_.GetSize() == 0)
            throw std::runtime_error("No Spinnaker cameras detected");

        cam_ = serial_.empty() ? cam_list_.GetByIndex(0) : cam_list_.GetBySerial(serial_);
        cam_->Init();
        INodeMap& nm = cam_->GetNodeMap();

        // The Bumblebee X auto-starts streaming on Init(). ComponentEnable
        // writes are silently ignored while the GenTL stream is open, so we
        // must force an AcquisitionStop and DSClose before configuring components.
        try {
            CCommandPtr stop = nm.GetNode("AcquisitionStop");
            if (IsAvailable(stop) && IsWritable(stop)) {
                stop->Execute();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        } catch (...) {}

        try {
            cam_->BeginAcquisition();
            cam_->EndAcquisition();
        } catch (...) {}

        set_enum(nm, "AcquisitionMode", "Continuous");

        // Deactivate Raw stream components to significantly reduce GenTL payload size.
        // This prevents network interface congestion on embedded ARM64 boards.
        const char* raw_sources[] = {"Sensor1", "Sensor2"};
        for (const auto& src : raw_sources) {
            set_enum(nm, "SourceSelector",    src);
            set_enum(nm, "ComponentSelector", "Raw");
            set_bool(nm, "ComponentEnable",   false);
        }

        // Activate only the Rectified and Disparity components required for this node.
        struct Comp { const char* src; const char* comp; };
        const Comp comps[] = {
            {"Sensor1", "Rectified"},
            {"Sensor2", "Rectified"},
            {"Sensor1", "Disparity"},
        };
        for (const auto& c : comps) {
            set_enum(nm, "SourceSelector",    c.src);
            set_enum(nm, "ComponentSelector", c.comp);
            set_bool(nm, "ComponentEnable",   true);
            // Readback so we can verify the write was accepted
            CBooleanPtr en = nm.GetNode("ComponentEnable");
            RCLCPP_INFO(get_logger(), "ComponentEnable[%s/%s] = %s",
                c.src, c.comp,
                (IsReadable(en) && en->GetValue()) ? "true" : "FALSE");
        }

        set_enum(nm, "SourceSelector",    "Sensor1");
        set_enum(nm, "ComponentSelector", "Rectified");
        set_enum(nm, "PixelFormat",       fmt_.c_str());
        RCLCPP_INFO(get_logger(), "PixelFormat → %s", fmt_.c_str());

        try {
            set_enum(nm, "StereoResolution", res_.c_str());
            RCLCPP_INFO(get_logger(), "StereoResolution → %s", res_.c_str());
        } catch (Spinnaker::Exception& e) {
            RCLCPP_WARN(get_logger(), "StereoResolution: %s", e.what());
        }

        try {
            set_bool(nm, "AcquisitionFrameRateEnable", true);
            CFloatPtr fr = nm.GetNode("AcquisitionFrameRate");
            if (IsWritable(fr)) {
                const double clamped = std::min(fps_, fr->GetMax());
                fr->SetValue(clamped);
                fps_ = clamped;
            }
        } catch (...) {}

        // Seed static calibration parameters from the nodemap.
        fallback_.focal_length = get_float(nm, "Scan3dFocalLength");
        fallback_.principal_u  = get_float(nm, "Scan3dPrincipalPointU");
        fallback_.principal_v  = get_float(nm, "Scan3dPrincipalPointV");
        fallback_.baseline     = get_float(nm, "Scan3dBaseline");
        fallback_.coord_scale  = get_float(nm, "Scan3dCoordinateScale");
        fallback_.coord_offset = get_float(nm, "Scan3dCoordinateOffset");

        // Sentinel used by the camera to mark pixels with no valid disparity;
        // needed so we can punch NaN holes in the depth image / point cloud.
        try {
            CBooleanPtr flag = nm.GetNode("Scan3dInvalidDataFlag");
            if (IsReadable(flag)) fallback_.invalid_flag = flag->GetValue();
            fallback_.invalid_value = static_cast<float>(get_float(nm, "Scan3dInvalidDataValue"));
        } catch (...) {}

        RCLCPP_INFO(get_logger(),
            "Nodemap intrinsics: f=%.2f  cx=%.2f  cy=%.2f  baseline=%.4f m  scale=%.6f  invalid_flag=%s  invalid_value=%.1f",
            fallback_.focal_length, fallback_.principal_u, fallback_.principal_v,
            fallback_.baseline, fallback_.coord_scale,
            fallback_.invalid_flag ? "true" : "false", fallback_.invalid_value);

        // The firmware leaves chunk data out of PayloadSize, which overflows the
        // buffers; disable it and use the nodemap intrinsics instead.
        disable_chunk_data();
        setup_device_link_throughput();

        // NewestOnly: drop late or incomplete frames instead of queueing them.
        try {
            INodeMap& nm_stream = cam_->GetTLStreamNodeMap();
            set_enum(nm_stream, "StreamBufferHandlingMode", "NewestOnly");
            RCLCPP_INFO(get_logger(), "StreamBufferHandlingMode → NewestOnly");
        } catch (...) {}

        try {
            CIntegerPtr ps = nm.GetNode("PayloadSize");
            if (IsReadable(ps))
                RCLCPP_INFO(get_logger(), "Optimized PayloadSize = %ld bytes (%.2f MB)",
                    static_cast<long>(ps->GetValue()), ps->GetValue() / 1048576.0);
        } catch (...) {}

        cam_->BeginAcquisition();
        RCLCPP_INFO(get_logger(), "Acquisition started successfully");
    }

    void disable_chunk_data()
    {
        try {
            if (!IsWritable(cam_->ChunkModeActive)) return;
            if (IsReadable(cam_->ChunkSelector)) {
                NodeList_t entries;
                cam_->ChunkSelector.GetEntries(entries);
                for (auto& entry : entries) {
                    CEnumEntryPtr ep = entry;
                    if (!IsReadable(ep)) continue;
                    cam_->ChunkSelector.SetIntValue(ep->GetValue());
                    if (IsAvailable(cam_->ChunkEnable) && IsWritable(cam_->ChunkEnable))
                        cam_->ChunkEnable.SetValue(false);
                }
            }
            cam_->ChunkModeActive.SetValue(false);
        } catch (...) {}
    }

    void setup_device_link_throughput()
    {
        try {
            if (!IsReadable(cam_->GevSCPSPacketSize) || !IsWritable(cam_->GevSCPSPacketSize)) return;
            const unsigned int maxGev = static_cast<unsigned int>(cam_->GevSCPSPacketSize.GetMax());
            unsigned int discovered = cam_->DiscoverMaxPacketSize();
            if (discovered == 0) discovered = 8228;
            const unsigned int pktSize = (discovered > maxGev) ? maxGev : discovered;
            cam_->GevSCPSPacketSize.SetValue(pktSize);

            if (!IsReadable(cam_->DeviceLinkCurrentThroughput) || !IsWritable(cam_->DeviceLinkThroughputLimit)) return;
            const int64_t current = cam_->DeviceLinkCurrentThroughput.GetValue();
            const int64_t minLimit = cam_->DeviceLinkThroughputLimit.GetMin();
            cam_->DeviceLinkThroughputLimit.SetValue(current < minLimit ? minLimit : current);
        } catch (...) {}
    }

    // ── Main Processing Loop ──────────────────────────────────────────────

    void timer_cb()
    {
        const uint64_t timeout_ms = static_cast<uint64_t>(std::max(3000.0, 3000.0 / fps_));
        ImageList imageList;
        try {
            imageList = cam_->GetNextImageSync(timeout_ms);
        } catch (Spinnaker::Exception& e) {
            RCLCPP_WARN(get_logger(), "GetNextImageSync failed: %s", e.what());
            return;
        }

        // RAII guard ensures buffers are released back to the GenTL pool regardless of exceptions
        struct ReleaseGuard {
            ImageList& il;
            ~ReleaseGuard() { try { il.Release(); } catch (...) {} }
        } guard{imageList};

        const size_t list_size = imageList.GetSize();
        if (list_size == 0) return;

        // First-frame diagnostic 
        if (!first_frame_logged_) {
            first_frame_logged_ = true;
            RCLCPP_INFO(get_logger(),
                "First ImageList received: %zu image(s)", list_size);
            for (size_t i = 0; i < list_size; ++i) {
                try {
                    ImagePtr img = imageList.GetByIndex(i);
                    if (img) {
                        RCLCPP_INFO(get_logger(),
                            "  [%zu] image_payload_type=%d  %zux%zu  incomplete=%s",
                            i,
                            static_cast<int>(img->GetImagePayloadType()),
                            img->GetWidth(), img->GetHeight(),
                            img->IsIncomplete() ? "YES" : "no");
                    }
                } catch (...) {}
            }
            // Expected: 6=RawSensor1 7=RawSensor2 8=RectSensor1 9=RectSensor2 10=Disparity
        }

        if (list_size == 0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Empty ImageList — component enables may not have taken effect");
            return;
        }

        // Throttled health check — reports how many of the 3 expected streams
        // are complete each cycle so we can spot persistent packet loss.
        {
            int n_complete = 0;
            for (size_t i = 0; i < list_size; ++i) {
                try {
                    ImagePtr img = imageList.GetByIndex(i);
                    if (img && !img->IsIncomplete()) ++n_complete;
                } catch (...) {}
            }
            if (n_complete < static_cast<int>(list_size)) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                    "Frame incomplete: %d/%zu images OK — possible packet loss",
                    n_complete, list_size);
            }
        }

        const auto now = get_clock()->now();
        FrameIntrinsics fi = fallback_;

        // Safely extract individual stream payloads from the GenTL multi-part buffer
        ImagePtr pLeft  = get_image(imageList, SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1, "RectSensor1");
        ImagePtr pRight = get_image(imageList, SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2, "RectSensor2");
        ImagePtr pDisp  = get_image(imageList, SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1, "DisparitySensor1");

        // OpenCV Display Matrices
        cv::Mat display_left, display_right, display_disp;
        cv::Size frame_size(0, 0);

        // Process Left Rectified
        if (pLeft && !pLeft->IsIncomplete()) {
            publish_rgb(pLeft, frame_id_, pub_left_, pub_linfo_, now, fi);
            if (debug_view_) {
                try {
                    cv::Mat cv_rgb(pLeft->GetHeight(), pLeft->GetWidth(), CV_8UC3, (void*)pLeft->GetData(), pLeft->GetStride());
                    cv::cvtColor(cv_rgb, display_left, cv::COLOR_RGB2BGR);
                    frame_size = display_left.size();
                } catch (...) {}
            }
        }

        // Process Right Rectified
        if (pRight && !pRight->IsIncomplete()) {
            publish_rgb(pRight, "navcam_right_optical_frame", pub_right_, pub_rinfo_, now, fi);
            if (debug_view_) {
                try {
                    cv::Mat cv_rgb(pRight->GetHeight(), pRight->GetWidth(), CV_8UC3, (void*)pRight->GetData(), pRight->GetStride());
                    cv::cvtColor(cv_rgb, display_right, cv::COLOR_RGB2BGR);
                    frame_size = display_right.size();
                } catch (...) {}
            }
        }

        // Process Disparity Map
        if (pDisp && !pDisp->IsIncomplete()) {
            if (speckle_enable_ && fi.coord_scale > 0.0) {
                Spinnaker::ImageUtilityStereo::FilterSpecklesFromImage(
                    pDisp, max_speckle_, speckle_thresh_, static_cast<float>(fi.coord_scale), fi.invalid_value);
            }
            publish_disparity(pDisp, frame_id_, pub_disp_, now);

            // Reconstruct metric depth (32FC1 matrix) and an organized XYZRGB
            // point cloud from the disparity, coloured with the left rectified image.
            publish_depth_and_cloud(pDisp, pLeft, frame_id_, now, fi);

            if (debug_view_) {
                try {
                    cv::Mat cv_disp16(pDisp->GetHeight(), pDisp->GetWidth(), CV_16UC1, (void*)pDisp->GetData(), pDisp->GetStride());
                    cv::Mat cv_disp8;
                    // Normalize 16-bit depth to 8-bit for visualization
                    cv::normalize(cv_disp16, cv_disp8, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    // Apply Jet Colormap (Blue = Far, Red = Near)
                    cv::applyColorMap(cv_disp8, display_disp, cv::COLORMAP_JET);
                    frame_size = display_disp.size();
                } catch (...) {}
            }
        }

        // ── Single Window OpenCV Concatenation (debug only) ──
        if (debug_view_ && frame_size.width > 0 && frame_size.height > 0) {
            // Allocate black placeholders for any dropped frames to prevent concatenation crashes
            if (display_left.empty()) display_left = cv::Mat::zeros(frame_size, CV_8UC3);
            if (display_right.empty()) display_right = cv::Mat::zeros(frame_size, CV_8UC3);
            if (display_disp.empty()) display_disp = cv::Mat::zeros(frame_size, CV_8UC3);

            // Overlay identification labels
            const int font = cv::FONT_HERSHEY_SIMPLEX;
            const cv::Scalar color(0, 255, 0); // Green text
            cv::putText(display_left, "Left Rectified", cv::Point(20, 40), font, 1.0, color, 2);
            cv::putText(display_right, "Right Rectified", cv::Point(20, 40), font, 1.0, color, 2);
            cv::putText(display_disp, "Disparity (Depth)", cv::Point(20, 40), font, 1.0, color, 2);

            // Concatenate horizontally [Left | Right | Disparity]
            cv::Mat combined;
            std::vector<cv::Mat> matrices = {display_left, display_right, display_disp};
            cv::hconcat(matrices, combined);

            // Scale down by 50% for display purposes (3 images horizontally is very wide)
            cv::Mat combined_resized;
            cv::resize(combined, combined_resized, cv::Size(), 0.5, 0.5);

            cv::imshow("Bumblebee X - Synchronized Pipeline", combined_resized);
            cv::waitKey(1); // Mandatory OpenCV GUI event loop update
        }
    }

    // ── Utility Methods ───────────────────────────────────────────────────

    ImagePtr get_image(ImageList& list, ImagePayloadType type, const char* label)
    {
        try {
            return list.GetByPayloadType(type);
        } catch (...) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s stream missing from multipart payload.", label);
            return nullptr;
        }
    }

    void publish_rgb(
        const ImagePtr& img, const std::string& frame_id,
        image_transport::Publisher& pub, rclcpp::Publisher<CI>::SharedPtr& info_pub,
        const rclcpp::Time& stamp, const FrameIntrinsics& fi)
    {
        const size_t w = img->GetWidth();
        const size_t h = img->GetHeight();
        const size_t stride = img->GetStride();
        const uint8_t* data = static_cast<const uint8_t*>(img->GetData());

        Img msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = frame_id;
        msg.width    = static_cast<uint32_t>(w);
        msg.height   = static_cast<uint32_t>(h);
        msg.encoding = "rgb8";
        msg.step     = static_cast<uint32_t>(w * 3);
        msg.data.resize(h * w * 3);

        for (size_t r = 0; r < h; ++r)
            std::memcpy(msg.data.data() + r * w * 3, data + r * stride, w * 3);

        pub.publish(msg);
        info_pub->publish(make_caminfo(stamp, frame_id, w, h, fi));
    }

    void publish_disparity(const ImagePtr& img, const std::string& frame_id, image_transport::Publisher& pub, const rclcpp::Time& stamp)
    {
        const size_t w = img->GetWidth();
        const size_t h = img->GetHeight();
        const size_t stride = img->GetStride();
        const uint8_t* data = static_cast<const uint8_t*>(img->GetData());

        Img msg;
        msg.header.stamp    = stamp;
        msg.header.frame_id = frame_id;
        msg.width    = static_cast<uint32_t>(w);
        msg.height   = static_cast<uint32_t>(h);
        msg.encoding = "16UC1";
        msg.step     = static_cast<uint32_t>(w * 2);
        msg.data.resize(h * w * 2);

        for (size_t r = 0; r < h; ++r)
            std::memcpy(msg.data.data() + r * w * 2, data + r * stride, w * 2);

        pub.publish(msg);
    }

    /**
     * @brief Reconstruct metric depth and an organized colour point cloud from disparity.
     *
     * Depth is recovered per pixel from the standard stereo relation
     *   disparity_px = raw * Scan3dCoordinateScale (+ offset)
     *   Z = focal_length * baseline / disparity_px
     *   X = (u - cx) * Z / f,  Y = (v - cy) * Z / f
     * in the left optical frame (Z forward, X right, Y down).
     *
     * Publishes:
     *   - a 32FC1 depth image (metres; NaN where disparity is invalid);
     *   - an organized (height x width) XYZRGB PointCloud2, coloured from the left
     *     rectified image when it is present and size-matched.
     */
    void publish_depth_and_cloud(
        const ImagePtr& pDisp, const ImagePtr& pLeft,
        const std::string& frame_id, const rclcpp::Time& stamp, const FrameIntrinsics& fi)
    {
        const size_t w = pDisp->GetWidth();
        const size_t h = pDisp->GetHeight();
        const size_t stride = pDisp->GetStride();
        const uint8_t* base = static_cast<const uint8_t*>(pDisp->GetData());

        const double f        = fi.focal_length;
        const double cx       = fi.principal_u;
        const double cy       = fi.principal_v;
        const double baseline = fi.baseline;
        const double scale    = fi.coord_scale;
        const double offset   = fi.coord_offset;
        const bool   have_calib = (f > 0.0 && baseline > 0.0 && scale > 0.0);

        if (!have_calib) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Missing stereo intrinsics — cannot reconstruct depth/points this frame");
            return;
        }

        const float NaN = std::numeric_limits<float>::quiet_NaN();

        // Colour source: left rectified, only if present, complete and size-matched.
        const uint8_t* left_data = nullptr;
        size_t left_stride = 0;
        if (pLeft && !pLeft->IsIncomplete() &&
            pLeft->GetWidth() == w && pLeft->GetHeight() == h) {
            left_data   = static_cast<const uint8_t*>(pLeft->GetData());
            left_stride = pLeft->GetStride();
        }

        // 32FC1 depth image (metres).
        Img depth;
        depth.header.stamp    = stamp;
        depth.header.frame_id = frame_id;
        depth.width    = static_cast<uint32_t>(w);
        depth.height   = static_cast<uint32_t>(h);
        depth.encoding = "32FC1";
        depth.step     = static_cast<uint32_t>(w * sizeof(float));
        depth.data.resize(h * w * sizeof(float));
        float* depth_ptr = reinterpret_cast<float*>(depth.data.data());

        // Organized XYZRGB cloud.
        PC2 cloud;
        cloud.header.stamp    = stamp;
        cloud.header.frame_id = frame_id;
        cloud.height       = static_cast<uint32_t>(h);
        cloud.width        = static_cast<uint32_t>(w);
        cloud.is_bigendian = false;
        cloud.is_dense     = false;   // carries NaNs for pixels without a valid range
        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2Fields(4,
            "x",   1, sensor_msgs::msg::PointField::FLOAT32,
            "y",   1, sensor_msgs::msg::PointField::FLOAT32,
            "z",   1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
        mod.resize(w * h);
        // PointCloud2Modifier::resize() collapses to height=1; restore the organized
        // (height x width) layout so the cloud stays image-aligned with the disparity.
        cloud.height   = static_cast<uint32_t>(h);
        cloud.width    = static_cast<uint32_t>(w);
        cloud.row_step = cloud.point_step * static_cast<uint32_t>(w);
        sensor_msgs::PointCloud2Iterator<float>   ix(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float>   iy(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float>   iz(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> irgb(cloud, "rgb");  // packed b,g,r,a

        const uint16_t invalid_raw = static_cast<uint16_t>(fi.invalid_value);

        for (size_t v = 0; v < h; ++v) {
            const uint16_t* drow = reinterpret_cast<const uint16_t*>(base + v * stride);
            const uint8_t*  lrow = left_data ? (left_data + v * left_stride) : nullptr;
            for (size_t u = 0; u < w; ++u, ++ix, ++iy, ++iz, ++irgb) {
                const uint16_t raw = drow[u];
                const double disp = raw * scale + offset;
                bool invalid = (disp <= 0.0);
                if (fi.invalid_flag && raw == invalid_raw) invalid = true;

                if (invalid) {
                    depth_ptr[v * w + u] = NaN;
                    *ix = *iy = *iz = NaN;
                    irgb[0] = irgb[1] = irgb[2] = irgb[3] = 0;
                    continue;
                }

                const double Z = f * baseline / disp;
                const double X = (static_cast<double>(u) - cx) * Z / f;
                const double Y = (static_cast<double>(v) - cy) * Z / f;

                depth_ptr[v * w + u] = static_cast<float>(Z);
                *ix = static_cast<float>(X);
                *iy = static_cast<float>(Y);
                *iz = static_cast<float>(Z);

                if (lrow) {   // left is rgb8; PointCloud2 rgb field is packed b,g,r,a
                    irgb[0] = lrow[u * 3 + 2];
                    irgb[1] = lrow[u * 3 + 1];
                    irgb[2] = lrow[u * 3 + 0];
                    irgb[3] = 0;
                } else {
                    irgb[0] = irgb[1] = irgb[2] = 200; irgb[3] = 0;
                }
            }
        }

        pub_depth_.publish(depth);
        pub_cloud_->publish(cloud);
    }

    CI make_caminfo(const rclcpp::Time& stamp, const std::string& frame_id, size_t w, size_t h, const FrameIntrinsics& fi) const
    {
        CI ci;
        ci.header.stamp    = stamp;
        ci.header.frame_id = frame_id;
        ci.width   = static_cast<uint32_t>(w);
        ci.height  = static_cast<uint32_t>(h);
        ci.distortion_model = "plumb_bob";
        if (fi.focal_length > 0.0) {
            ci.k = {fi.focal_length, 0.0, fi.principal_u, 0.0, fi.focal_length, fi.principal_v, 0.0, 0.0, 1.0};
            const double tx = (frame_id.find("right") != std::string::npos) ? -fi.focal_length * fi.baseline : 0.0;
            ci.p = {fi.focal_length, 0.0, fi.principal_u, tx, 0.0, fi.focal_length, fi.principal_v, 0.0, 0.0, 0.0, 1.0, 0.0};
        }
        return ci;
    }

    // ── Dynamic reconfigure ───────────────────────────────────────────────

    // Log which tunable GenICam controls this camera exposes.
    void probe_controls()
    {
        if (!cam_) return;
        INodeMap& nm = cam_->GetNodeMap();
        const char* nodes[] = {
            "ExposureAuto", "ExposureTime", "GainAuto", "Gain",
            "BalanceWhiteAuto", "Gamma", "BlackLevel",
            "Focus", "FocusAuto", "Aperture", "Iris",
        };
        std::string supported, missing;
        for (const char* n : nodes) {
            if (node_writable(nm, n)) { supported += n; supported += " "; }
            else                      { missing   += n; missing   += " "; }
        }
        RCLCPP_INFO(get_logger(), "Dynamic controls writable on this camera: %s",
                    supported.empty() ? "(none)" : supported.c_str());
        RCLCPP_INFO(get_logger(), "Not available on this camera: %s", missing.c_str());
    }

    // Push the current values of the tunable parameters to the camera once at
    // startup (so launch-provided overrides take effect), reusing the same logic
    // as the live callback.
    void apply_current_controls()
    {
        on_params_set({
            get_parameter("exposure_auto"),  get_parameter("exposure_time"),
            get_parameter("gain_auto"),      get_parameter("gain"),
            get_parameter("white_balance_auto"),
            get_parameter("gamma"),          get_parameter("black_level"),
        });
    }

    // Live parameter callback: applies image controls to the camera nodemap.
    // Runs in the same (single-threaded) executor as timer_cb, so there is no
    // concurrent nodemap access. Startup-only params (serial/resolution/...) are
    // accepted but not re-applied here - they need a restart.
    rcl_interfaces::msg::SetParametersResult
    on_params_set(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        if (!cam_) {
            result.successful = false;
            result.reason = "camera not initialized";
            return result;
        }
        INodeMap& nm = cam_->GetNodeMap();

        for (const auto& p : params) {
            const std::string& name = p.get_name();
            try {
                if (name == "exposure_auto") {
                    set_enum(nm, "ExposureAuto", p.as_bool() ? "Continuous" : "Off");
                } else if (name == "exposure_time") {
                    if (p.as_double() > 0.0) {
                        set_enum(nm, "ExposureAuto", "Off");
                        if (!write_float(nm, "ExposureTime", p.as_double()))
                            RCLCPP_WARN(get_logger(), "ExposureTime not writable on this camera");
                    }
                } else if (name == "gain_auto") {
                    set_enum(nm, "GainAuto", p.as_bool() ? "Continuous" : "Off");
                } else if (name == "gain") {
                    if (p.as_double() >= 0.0) {
                        set_enum(nm, "GainAuto", "Off");
                        if (!write_float(nm, "Gain", p.as_double()))
                            RCLCPP_WARN(get_logger(), "Gain not writable on this camera");
                    }
                } else if (name == "white_balance_auto") {
                    set_enum(nm, "BalanceWhiteAuto", p.as_bool() ? "Continuous" : "Off");
                } else if (name == "gamma") {
                    if (p.as_double() >= 0.0) {
                        set_bool(nm, "GammaEnable", true);
                        if (!write_float(nm, "Gamma", p.as_double()))
                            RCLCPP_WARN(get_logger(), "Gamma not writable on this camera");
                    }
                } else if (name == "black_level") {
                    if (p.as_double() >= 0.0 && !write_float(nm, "BlackLevel", p.as_double()))
                        RCLCPP_WARN(get_logger(), "BlackLevel not writable on this camera");
                } else if (name == "enable_speckle_filter") {
                    speckle_enable_ = p.as_bool();
                } else if (name == "max_speckle_size") {
                    max_speckle_ = static_cast<int>(p.as_int());
                } else if (name == "speckle_threshold") {
                    speckle_thresh_ = static_cast<float>(p.as_double());
                }
                // Other declared params are startup-only; accepted without action.
            } catch (Spinnaker::Exception& e) {
                result.successful = false;
                result.reason = name + ": " + e.what();
                RCLCPP_WARN(get_logger(), "Failed to apply %s: %s", name.c_str(), e.what());
            }
        }
        return result;
    }

    // ── Class Members ──
    SystemPtr  sys_;
    CameraList cam_list_;
    CameraPtr  cam_;

    std::string serial_, res_, fmt_, frame_id_{"camera_optical_frame"};
    double fps_{2.0};
    bool   speckle_enable_{true};
    int    max_speckle_{1000};
    float  speckle_thresh_{3.0f};
    bool   debug_view_{false};

    FrameIntrinsics fallback_;
    bool first_frame_logged_{false};

    image_transport::Publisher pub_left_, pub_right_, pub_disp_, pub_depth_;
    rclcpp::Publisher<PC2>::SharedPtr pub_cloud_;
    rclcpp::Publisher<CI>::SharedPtr  pub_linfo_, pub_rinfo_;
    rclcpp::TimerBase::SharedPtr      timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BumbleBeeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}