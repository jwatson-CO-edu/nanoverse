# Setup
1. `dotnet new console -n Paraboloid -o 02_ParabolicDish`
1. Nav to folder
1. `dotnet new gitignore`
1. `dotnet add Paraboloid.csproj package OpenTK`
1. `dotnet add Paraboloid.csproj package SVG.NET`
1. `dotnet add Paraboloid.csproj package MathNet.Numerics`
# Build
1. Dev
1. `dotnet publish -c Release -r linux-x64 --self-contained false /p:DebugType=None /p:PublishSingleFile=true /p:AssemblyName="EXECNAME.out" -o ~/OUTPUT/DIRECTORY/`