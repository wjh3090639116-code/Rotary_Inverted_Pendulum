param([string[]]$Tests=@())
& python "$PSScriptRoot/run_modelsim.py" @Tests
exit $LASTEXITCODE
