from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout

# Windows 交付目标 VS 版本，切换时修改此变量。macOS 忽略此项。
# 可选值: "vs2015" | "vs2017" | "vs2022"
WINDOWS_TARGET_VS = "vs2015"

# 依赖版本表
#   vs2015: MSVC 14.0，要求 C++11，部分 C++14
#   vs2017+: MSVC 14.1+，C++14/17 完整支持，可用最新版
_DEPS = {
    "vs2015": {
        "spdlog": "spdlog/1.8.5",
        "eigen":  "eigen/3.3.9",
        "fmt":    "fmt/7.1.3",
        "boost":  "boost/1.79.0",
    },
    "modern": {
        "spdlog": "spdlog/1.12.0",
        "eigen":  "eigen/3.4.0",
        "fmt":    "fmt/10.2.1",
        "boost":  "boost/1.83.0",
    },
}


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        is_vs2015_target = (
            self.settings.os == "Windows" and WINDOWS_TARGET_VS == "vs2015"
        )
        deps = _DEPS["vs2015"] if is_vs2015_target else _DEPS["modern"]

        self.requires(deps["spdlog"])
        self.requires(deps["eigen"])
        self.requires(deps["fmt"], override=True)
        self.requires(deps["boost"])
        self.requires("eventpp/0.1.3")
        self.requires("gtest/1.12.1", test=True)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
