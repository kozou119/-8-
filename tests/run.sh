#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
build_dir=$(mktemp -d /tmp/zumo-controller-tests.XXXXXX)
trap 'rm -rf "$build_dir"' EXIT
${CXX:-g++} -std=c++11 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. tests/controller_test.cpp -o "$build_dir/controller_test"
"$build_dir/controller_test"
${CXX:-g++} -std=c++11 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -g -Itests/fakes -I. tests/board_io_test.cpp -o "$build_dir/board_io_test"
"$build_dir/board_io_test"
${CXX:-g++} -std=c++11 -Wall -Wextra -Werror -pedantic -fsanitize=address,undefined -fno-omit-frame-pointer -g -I. tests/navigation_math_test.cpp -o "$build_dir/navigation_math_test"
"$build_dir/navigation_math_test"
java --module jdk.compiler/com.sun.tools.javac.Main -d "$build_dir" visualization/ContestDashboard/TelemetryFrame.java tests/TelemetryFrameTest.java
java -cp "$build_dir" TelemetryFrameTest
