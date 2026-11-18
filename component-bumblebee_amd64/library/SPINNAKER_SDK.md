# Spinnaker SDK (not included)

The Bumblebee X camera needs Teledyne FLIR's **Spinnaker SDK 4.3.0.189**. It is proprietary
software under the FLIR EULA, so it is not distributed with this repository.

1. Create a free account and download the SDK for **Ubuntu 24.04, amd64** from
   <https://www.teledynevisionsolutions.com/products/spinnaker-sdk/>.
2. Extract it and copy all its `.deb` packages into this folder
   (`libgentl_*`, `libspinnaker*_*`, `libspinvideo*_*`, `spinnaker_*`, `spinnaker-doc_*`,
   `spinupdate*_*`, `spinview-qt*_*`).
3. Build the image. The Dockerfile installs the packages with `dpkg` and accepts the FLIR
   EULA non-interactively, so read the EULA before building.

Files placed here are ignored by git.
