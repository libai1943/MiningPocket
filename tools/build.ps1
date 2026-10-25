param([string]$Compiler='clang++')
$project=Split-Path -Parent $PSScriptRoot
$build=Join-Path $project 'build'
if (-not (Test-Path -LiteralPath (Join-Path $project 'benchmark/case_001.json'))) {
 Expand-Archive -LiteralPath (Join-Path $project 'benchmark/cases.zip') -DestinationPath (Join-Path $project 'benchmark')
}
New-Item -ItemType Directory -Path $build -Force | Out-Null
$src=Get-ChildItem -LiteralPath (Join-Path $project 'src') -Filter '*.cpp' | ForEach-Object FullName
& $Compiler -std=c++17 -O3 -DNDEBUG -static -Wall -Wextra @src -o (Join-Path $build 'miningpocket.exe')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
$info=[ordered]@{ compiler=(& $Compiler --version | Out-String); flags='-std=c++17 -O3 -DNDEBUG -static -Wall -Wextra'; threads=1 }
$info | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $build 'build_info.json') -Encoding utf8
