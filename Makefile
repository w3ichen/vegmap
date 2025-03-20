# PHONY prevents is up to date check
.PHONY: build

ifeq ($(shell uname -s),Darwin)
build:
	colcon build --symlink-install
	@echo 'export PATH=/opt/homebrew/bin/ign:/opt/homebrew/bin/gz:$${PATH}' >> install/setup.bash
	@echo 'export PATH=/opt/homebrew/bin/ign:/opt/homebrew/bin/gz:$${PATH}' >> install/setup.zsh
	@echo 'export PYTHONPATH=$${CONDA_PREFIX}/lib/python3.10/site-packages:$${PYTHONPATH}' >> install/setup.bash
	@echo 'export PYTHONPATH=$${CONDA_PREFIX}/lib/python3.10/site-packages:$${PYTHONPATH}' >> install/setup.zsh
else
build:
	colcon build --symlink-install
endif

clean:
	rm -rf build install log