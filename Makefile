# PHONY prevents is up to date check
.PHONY: build

build:
	colcon build --symlink-install
clean:
	rm -rf build install log