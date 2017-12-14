deb-pkg:
	tar caf ../ros-kinetic-aero-telemetry-simple_0.1.0.orig.tar.gz . --exclude debian
	dpkg-buildpackage||exit 0
	mkdir -p ../debian-packages
	mv ../ros-kinetic-aero-telemetry-simple_0.1.0-0xenial_amd64.deb ../debian-packages
	rm ../*.dsc ../*.changes ../*.gz ../*.xz

