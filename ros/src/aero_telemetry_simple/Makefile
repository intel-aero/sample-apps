INSTALL_DIR=../debian-packages
ROS_LIB_DIR=/opt/ros/kinetic/lib
ROS_PKG_NAME=aero_telemetry_simple
ROS_PKG_PATH=/opt/ros/kinetic/share
DEB_PKG_NAME=ros-kinetic-aero-telemetry-simple
VER=0.1.0
DEB_PKG_PATH=$(INSTALL_DIR)/$(DEB_PKG_NAME)_$(VER)
OBJ_DIR=obj-`arch`-linux-gnu

define boldecho
      @tput bold
      @echo $1
      @tput sgr0
endef

deb-pkg:
	@tar caf ../$(DEB_PKG_NAME)_$(VER).orig.tar.gz . --exclude debian
	@$(call boldecho, "Created tar file `readlink -f ../$(DEB_PKG_NAME)_$(VER).orig.tar.gz`.")
	@# compose debian .install file
	@echo "$(OBJ_DIR)/devel/lib/$(ROS_PKG_NAME) $(ROS_LIB_DIR)" > debian/$(DEB_PKG_NAME).install
	@echo "launch $(ROS_PKG_PATH)/$(ROS_PKG_NAME)" >> debian/$(DEB_PKG_NAME).install
	@# debian build
	dpkg-buildpackage -uc -us
	@echo "Built debain package: $(DEB_PKG_NAME)_$(VER)."
	@mkdir -p $(DEB_PKG_PATH) # debian package dir for each ROS example
	@mv ../$(DEB_PKG_NAME)_$(VER)* $(DEB_PKG_PATH)
	@$(call boldecho, "Package is now available at $(abspath $(DEB_PKG_PATH)/$(DEB_PKG_NAME)_$(VER)-0xenial_amd64.deb)\n")
