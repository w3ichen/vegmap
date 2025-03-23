# PHONY prevents is up to date check
.PHONY: build rosdep clean plan

plan:
	colcon build --symlink-install --packages-select planner

ifeq ($(shell uname -s),Darwin)
build:
	colcon build --symlink-install --cmake-args -DPython3_FIND_VIRTUALENV=ONLY --packages-skip spacenav
	@echo 'export PATH=/opt/homebrew/bin/ign:/opt/homebrew/bin/gz:$${PATH}' >> install/setup.bash
	@echo 'export PATH=/opt/homebrew/bin/ign:/opt/homebrew/bin/gz:$${PATH}' >> install/setup.zsh
	@echo 'export PYTHONPATH=$${CONDA_PREFIX}/lib/python3.10/site-packages:$${PYTHONPATH}' >> install/setup.bash
	@echo 'export PYTHONPATH=$${CONDA_PREFIX}/lib/python3.10/site-packages:$${PYTHONPATH}' >> install/setup.zsh
	@echo 'export GZ_SIM_RESOURCE_PATH=install/clearpath_gz/share/clearpath_gz/worlds' >> install/setup.zsh
	@echo 'export IGN_GAZEBO_RESOURCE_PATH=install/clearpath_gz/share/clearpath_gz/worlds' >> install/setup.zsh
else
build:
	colcon build --symlink-install
endif

rosdep:
	rosdep install --from-paths src --ignore-src -y

clean:
	rm -rf build install log