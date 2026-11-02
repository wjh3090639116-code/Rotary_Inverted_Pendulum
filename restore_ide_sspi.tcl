# 工程辅助脚本：维持实现流程与引脚选项的一致性。
set config_file [file join [file dirname [file normalize [info script]]] impl Rotary_Inverted_Pendulum_process_config.json]
if {[file exists $config_file]} {
    set config_stream [open $config_file r]
    set config_data [read $config_stream]
    close $config_stream
    if {![regsub {"SSPI"[[:space:]]*:[[:space:]]*(false|true)} $config_data {"SSPI": true} config_data]} {
        error "SSPI field missing in IDE configuration"
    }
    set config_stream [open $config_file w]
    puts -nonewline $config_stream $config_data
    close $config_stream
    puts "IDE configuration synchronized: Use SSPI as regular IO = true"
}
