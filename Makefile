.PHONY: build test demo smoke eval-test benchmark-test benchmark clean

build:
	. /opt/ros/jazzy/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

test:
	. /opt/ros/jazzy/setup.sh && . install/setup.sh && colcon test --event-handlers console_direct+ && colcon test-result --verbose

demo:
	. /opt/ros/jazzy/setup.sh && . install/setup.sh && ros2 launch vif_bringup synthetic_demo.launch.py

smoke:
	./tools/benchmark/run_synthetic_smoke.sh

eval-test:
	python3 -m pytest evaluation/tests

benchmark-test:
	python3 -m unittest discover -s tools/benchmark/tests -v

benchmark:
	@test -n "$(DATASET)" || (echo "Set DATASET=/path/to/EuRoC/sequence" >&2; exit 2)
	@test -n "$(VOCABULARY)" || (echo "Set VOCABULARY=/path/to/ORBvoc.txt" >&2; exit 2)
	@test -n "$(SETTINGS)" || (echo "Set SETTINGS=/path/to/EuRoC.yaml" >&2; exit 2)
	./tools/benchmark/run_portfolio_benchmark.sh \
		--dataset "$(DATASET)" --vocabulary "$(VOCABULARY)" --settings "$(SETTINGS)"

clean:
	rm -rf build install log
