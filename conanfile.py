from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires("spdlog/1.8.5")
        self.requires("eigen/3.3.9")
        self.requires("fmt/7.1.3", override=True)
        self.requires("boost/1.79.0")
        self.requires("eventpp/0.1.3")
        self.requires("gtest/1.11.0", test=True)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
