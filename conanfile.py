from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout

_BASE_DEPS_VS2015 = {
    "eigen": "eigen/3.3.9",
    "boost": "boost/1.74.0",
    "nanoflann": "nanoflann/1.3.2",
    "nlohmann_json": "nlohmann_json/3.11.3",
}

_BASE_DEPS_MODERN = {
    "eigen": "eigen/3.4.0",
    "boost": "boost/1.83.0",
    "nanoflann": "nanoflann/1.6.0",
    "nlohmann_json": "nlohmann_json/3.11.3",
}

_LOG_DEPS_NON_WINDOWS = {
    "spdlog": "spdlog/1.12.0",
    "fmt": "fmt/10.2.1",
}

_VIS_DEPS_NON_WINDOWS = {
    "imgui": "imgui/1.90.5",
    "implot": "implot/0.16",
    "glfw": "glfw/3.4",
}

_GTEST_VERSION_VS2015 = "gtest/1.8.1"
_GTEST_VERSION_MODERN = "gtest/1.12.1"


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
        "glfw/*:shared": False,
        "imgui/*:shared": False,
        "implot/*:shared": False,
        "boost/*:header_only": True,
        "imgui/*:with_glfw": True,
        "imgui/*:with_opengl3": True,
    }

    def _is_windows(self):
        return str(self.settings.os) == "Windows"

    def _is_vs2015_target(self):
        if not self._is_windows():
            return False

        compiler = str(self.settings.compiler)
        version = str(self.settings.compiler.version)
        return (
            (compiler == "msvc" and version == "190")
            or (compiler == "Visual Studio" and version == "14")
        )

    def _base_deps(self):
        return _BASE_DEPS_VS2015 if self._is_vs2015_target() else _BASE_DEPS_MODERN

    def requirements(self):
        deps = self._base_deps()

        self.requires(deps["eigen"])
        self.requires(deps["boost"])
        self.requires(deps["nanoflann"])
        self.requires(deps["nlohmann_json"])

        # macOS/Linux 保留调试日志能力，Windows 全平台关闭日志依赖。
        if not self._is_windows():
            self.requires(_LOG_DEPS_NON_WINDOWS["spdlog"])
            self.requires(_LOG_DEPS_NON_WINDOWS["fmt"], override=True)

        # 可视化依赖（macOS/Linux dev 工具，Windows 不安装）
        if not self._is_windows():
            self.requires(_VIS_DEPS_NON_WINDOWS["imgui"])
            self.requires(_VIS_DEPS_NON_WINDOWS["implot"])
            self.requires(_VIS_DEPS_NON_WINDOWS["glfw"])

    def build_requirements(self):
        if self.options.enable_testing:
            gtest_version = _GTEST_VERSION_VS2015 if self._is_vs2015_target() else _GTEST_VERSION_MODERN
            self.test_requires(gtest_version)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
