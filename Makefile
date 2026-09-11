.PHONY: build
build:
	cmake -G Ninja -S . -B build/plain \
		-DCMAKE_BUILD_TYPE=Release \
		-DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
		-DCMAKE_POSITION_INDEPENDENT_CODE=ON \
		-DRKO_SLAM_BUILD_TESTS=ON
	cmake --build build/plain
	touch build/COLCON_IGNORE

.PHONY: test
test: build
	ctest --test-dir build/plain --output-on-failure

.PHONY: clean
clean:
	rm -rf build
