from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout

_BASE_DEPS = {
    "vs2015": {
        "eigen":  "eigen/3.3.9",
        "boost":  "boost/1.74.0",
        "nanoflann": "nanoflann/1.3.2",
    },
    "modern": {
        "eigen":  "eigen/3.4.0",
        "boost":  "boost/1.83.0",
        "nanoflann": "nanoflann/1.6.0",
    },
}

_MAC_LOG_DEPS = {
    "spdlog": "spdlog/1.12.0",
    "fmt": "fmt/10.2.1",
}


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    options = {
        "enable_testing": [True, False],
    }

    default_options = {
        "enable_testing": False,
        "boost/*:header_only": True,
        "imgui/*:with_glfw": True,
        "imgui/*:with_opengl3": True
    }

    def _is_windows(self):
        return str(self.settings.os) == "Windows"

    def _is_vs2015_target(self):
        if not self._is_windows():
            return False

        compiler = str(self.settings.compiler)
        version = str(self.settings.compiler.version)
        return ((compiler == "msvc" and version == "190") or
                (compiler == "Visual Studio" and version == "14"))

    def requirements(self):
        deps = _BASE_DEPS["vs2015"] if self._is_vs2015_target() else _BASE_DEPS["modern"]

        self.requires(deps["eigen"])
        self.requires(deps["boost"])
        self.requires("eventpp/0.1.3")
        self.requires(deps["nanoflann"])

        # macOS/Linux 保留调试日志能力，Windows 全平台关闭 spdlog/fmt。
        if not self._is_windows():
            self.requires(_MAC_LOG_DEPS["spdlog"])
            self.requires(_MAC_LOG_DEPS["fmt"], override=True)

        # 可视化依赖（macOS/Linux dev 工具，Windows 不安装）
        if not self._is_windows():
            self.requires("imgui/1.90.5")
            self.requires("implot/0.16")
            self.requires("glfw/3.4")

    def build_requirements(self):
        if self.options.enable_testing:
            if self._is_vs2015_target():
                self.test_requires("gtest/1.8.1")
            else:
                self.test_requires("gtest/1.12.1")

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
