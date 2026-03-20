from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        # Windows 使用 VS2015 (MSVC 14.0) 兼容版本；其他平台使用最新稳定版
        is_windows = self.settings.os == "Windows"

        self.requires("spdlog/1.8.5"  if is_windows else "spdlog/1.12.0")
        self.requires("eigen/3.3.9"   if is_windows else "eigen/3.4.0")
        self.requires("fmt/7.1.3"     if is_windows else "fmt/10.2.1", override=True)
        self.requires("boost/1.79.0"  if is_windows else "boost/1.83.0")
        self.requires("eventpp/0.1.3")
        self.requires("gtest/1.12.1", test=True)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
