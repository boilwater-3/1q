from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout

_BASE_DEPS = {
    "eigen": "eigen/3.4.0",
    "boost": "boost/1.85.0",
    "nanoflann": "nanoflann/1.3.2",
    "flatbuffers": "flatbuffers/1.12.0",
    "zlib": "zlib/1.3.1",
}

_JSBSIM_DEPS_NON_WINDOWS = {
    "jsbsim": "jsbsim/1.3.1",
}

_LOG_DEPS_NON_WINDOWS = {
    "spdlog": "spdlog/1.12.0",
    "fmt": "fmt/10.2.1",
}

_GTEST_VERSION = "gtest/1.12.1"


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "enable_testing": [True, False],
    }

    default_options = {
        "enable_testing": False,
        # 显式固定第三方链接形态，避免依赖 recipe 默认值导致构建行为漂移。
        "spdlog/*:shared": False,
        "fmt/*:shared": False,
        "gtest/*:shared": False,
        "zlib/*:shared": False,
        "boost/*:header_only": True,
        "flatbuffers/*:header_only": True,
    }

    def _is_windows(self):
        return str(self.settings.os) == "Windows"

    def requirements(self):
        self.requires(_BASE_DEPS["eigen"])
        self.requires(_BASE_DEPS["boost"])
        self.requires(_BASE_DEPS["nanoflann"])
        self.requires(_BASE_DEPS["flatbuffers"])
        self.requires(_BASE_DEPS["zlib"])

        # macOS/Linux 保留调试日志能力，Windows 全平台关闭日志依赖。
        if not self._is_windows():
            self.requires(_LOG_DEPS_NON_WINDOWS["spdlog"])
            self.requires(_LOG_DEPS_NON_WINDOWS["fmt"], override=True)
            # macOS 开发使用 conan 预编译的 JSBSim（Windows/VS2015 从 third_party 源码构建）
            self.requires(_JSBSIM_DEPS_NON_WINDOWS["jsbsim"])

            # SAR HDF5 输出（始终开启）
            self.requires("highfive/2.10.0")

    def build_requirements(self):
        if self.options.enable_testing:
            self.test_requires(_GTEST_VERSION)

    def generate(self):
        CMakeDeps(self).generate()
        tc = CMakeToolchain(self)
        tc.generate()

    def layout(self):
        cmake_layout(self)
