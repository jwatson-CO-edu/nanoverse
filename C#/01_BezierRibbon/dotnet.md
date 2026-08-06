1. `dotnet new console -n Ribbon -o 01_BezierRibbon`
1. Nav to folder
1. `dotnet new gitignore`
1. Dev
1. `dotnet publish -c Release -r linux-x64 --self-contained false /p:DebugType=None /p:PublishSingleFile=true /p:AssemblyName="EXECNAME.out" -o ~/OUTPUT/DIRECTORY/`