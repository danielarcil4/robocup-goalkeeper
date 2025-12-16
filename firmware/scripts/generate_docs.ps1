param()

if (-not (Get-Command doxygen -ErrorAction SilentlyContinue)) {
    Write-Error "doxygen not found in PATH. Install doxygen and try again."
    exit 1
}

doxygen Doxyfile
Write-Host "Doxygen finished. Open docs/html/index.html to view the docs."