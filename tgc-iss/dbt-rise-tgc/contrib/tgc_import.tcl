#############################################################################
#
#############################################################################
proc getScriptDirectory {} {
    set dispScriptFile [file normalize [info script]]
    set scriptFolder [file dirname $dispScriptFile]
    return $scriptFolder
}
if { $::env(SNPS_VP_PRODUCT) == "PAULTRA" } {
    set hardware /HARDWARE/HW/HW
} else {
    set hardware /HARDWARE
}

set scriptDir [getScriptDirectory]
set top_design_name core_complex
set clocks clk_i
set resets rst_i
set model_prefix "i_"
set model_postfix ""

::pct::new_project
::pct::open_library TLM2_PL
::pct::clear_systemc_defines
::pct::clear_systemc_include_path
::pct::add_to_systemc_include_path $::env(TGFS_INSTALL_ROOT)/include
::pct::set_import_protocol_generation_flag false
::pct::set_update_existing_encaps_flag true
::pct::set_dynamic_port_arrays_flag true
::pct::set_import_scml_properties_flag true
::pct::load_modules --set-category modules tgc_import.cc

# Set Port Protocols correctly
set block ${top_design_name}
foreach clock ${clocks} {
	::pct::set_block_port_protocol --set-category SYSTEM_LIBRARY:$block/${clock} SYSTEM_LIBRARY:CLOCK
}
foreach reset ${resets} {
    ::pct::set_block_port_protocol --set-category SYSTEM_LIBRARY:$block/${reset} SYSTEM_LIBRARY:RESET
}
::pct::set_encap_port_array_size SYSTEM_LIBRARY:$block/local_irq_i 16

# Set compile settings and look
set block SYSTEM_LIBRARY:${top_design_name}
::pct::set_encap_build_script $block/${top_design_name} $scriptDir/build.tcl
::pct::set_background_color_rgb $block 255 255 255 255
::pct::create_instance SYSTEM_LIBRARY:${top_design_name}  ${hardware} ${model_prefix}${top_design_name}${model_postfix} ${top_design_name} 

# export the result as component
::pct::export_system_library ${top_design_name}  ${top_design_name}.xml
