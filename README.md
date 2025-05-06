# VBeamRT
A ray-tracer showcasing iterative improvements and optimizations.
# How to compile
1. Install [.NET 8 SDK](https://dotnet.microsoft.com/en-us/download/dotnet/8.0)
<br/>(If you already have a newer version installed, you may change the version to the installed one in the .csproj file)

2. Open folder containing .sln in terminal

3. enter **dotnet build -c Release**

4. DONE. Executable binary should be in **/VBeamRT/bin/Release/net8.0**

The application uses vulkan for image display, rendering is done fully on CPU.
MacOS is untested but should work. Windows
