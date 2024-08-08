echo off

set DIR="%~dp0\applications"
for /R %DIR% %%f in (*.c,*.h) do (
    echo %%f
    clang-format.exe -i -style=file %%f
)

exit
