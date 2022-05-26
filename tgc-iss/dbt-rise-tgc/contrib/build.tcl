namespace eval Specification {
    proc buildproc { args } {
        global env
        variable installDir
        variable compiler
        variable compiler [::scsh::get_backend_compiler]
        #  set target $machine
        set target [::scsh::machine]
        set linkerOptions ""
        set preprocessorOptions ""
        set libversion $compiler
        switch -exact -- $target {
            "linux" {
            	set install_dir $::env(TGFS_INSTALL_ROOT)
                set incldir "${install_dir}/include"
                set libdir "${install_dir}/lib64"
                set preprocessorOptions [concat $preprocessorOptions "-I${incldir}"]
                # Set the Linker paths.
                set linkerOptions [concat $linkerOptions "-Wl,-rpath,${libdir} -L${libdir} -ldbt-rise-tgc_sc"]
            }
            default {
               puts stderr "ERROR: \"$target\" is not supported, [::scsh::version]"
               return
            }
        }
        ::scsh::cwr_append_ipsimbld_opts preprocessor "$preprocessorOptions"
        ::scsh::cwr_append_ipsimbld_opts linker       "$linkerOptions"
    }
    ::scsh::add_build_callback [namespace current]::buildproc
}
