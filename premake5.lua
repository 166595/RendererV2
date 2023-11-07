workspace "Renderer Project V2"
    configurations { "Debug", "Release" }

project "Renderer"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
    targetdir "bin/%{cfg.buildcfg}"
    

    files { "src/*.h", "src/*.cpp" }

    includedirs { "src" }

    filter "configurations:Debug"
        defines { "DEBUG" }
        symbols "On"

    filter "configurations:Release"
        defines { "NDEBUG" }
        optimize "On"