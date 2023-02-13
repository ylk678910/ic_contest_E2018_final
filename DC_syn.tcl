read_verilog TPA.v
current_design TPA
link

source -cho -verbose TPA.sdc

compile -map_effort high -area_effort high
compile -map_effort high -area_effort high -inc

write -format ddc -hierarchy -output "TPA_syn.ddc"
write_sdf TPA_syn.sdf
write_file -format verilog -hierarchy -output TPA_syn.v
report_area > area.log
report_timing > timing.log
report_qor > TPA_syn.qor