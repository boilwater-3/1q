from conan import ConanFile
from conan.tools.cmake import CMakeDeps, CMakeToolchain, cmake_layout


class OneQConan(ConanFile):
    name = "1q"
    version = "0.1"
    settings = "os", "compiler", "build_type", "arch"

    def requirements(self):
        self.requires("spdlog/1.12.0")
        self.requires("eigen/3.4.0")
        self.requires("sophus/1.22.10")
        self.requires("fmt/10.2.1", override=True)
        self.requires("boost/1.83.0")
        self.requires("eventpp/0.1.3")
        self.requires("gtest/1.12.1", test=True)

    def generate(self):
        CMakeDeps(self).generate()
        CMakeToolchain(self).generate()

    def layout(self):
        cmake_layout(self)
