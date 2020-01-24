package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "xtask",
    srcs = [
        "src/cilantro_track_task.cpp",
        "src/gravcomp_task.c",
        "src/impedance_test_task.c",
        "src/initUserTasks.c",
        "src/sample_task.c",
        "src/sample_task_cpp.cpp",
        "src/sm_controllers.c",
        "src/sm_task.c",
        "src/test_task.c",
    ],
    includes = [
        "include",
    ],
    deps = [
        "//experimental/users/sschaal/SL/SL:SLcommon",
        "//experimental/users/sschaal/SL/SL:SLtask",
        "//experimental/users/sschaal/SL/panda",
        "//experimental/users/sschaal/SL/panda:panda_task",
        "//experimental/users/sschaal/SL/panda:xmotor",
        "//experimental/users/sschaal/SL/panda:xpanda",
        "//experimental/users/sschaal/SL/panda:xpest",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)

cc_binary(
    name = "xopengl",
    srcs = [
        "src/initUserGraphics.c",
    ],
    includes = [
        "include",
    ],
    deps = [
        "//experimental/users/sschaal/SL/SL:SLcommon",
        "//experimental/users/sschaal/SL/SL:SLopenGL",
        "//experimental/users/sschaal/SL/panda",
        "//experimental/users/sschaal/SL/panda:panda_openGL",
        "//experimental/users/sschaal/SL/utilities:utility",
        "//third_party/freeglut:freeglut_base",
        "//third_party/glu:native",
        "//third_party/Xorg:Xorg_shared",	
    ],
)

cc_binary(
    name = "xsimulation",
    srcs = [
        "src/initUserSimulation.c",
    ],
    includes = [
        "include",
    ],
    deps = [
        "//experimental/users/sschaal/SL/SL:SLcommon",
        "//experimental/users/sschaal/SL/SL:SLsimulation",
        "//experimental/users/sschaal/SL/panda",
        "//experimental/users/sschaal/SL/panda:panda_simulation",
        "//experimental/users/sschaal/SL/utilities:utility",
    ],
)

genrule(
    name = "install_xtask",
    srcs = ["xtask"],
    outs = ["x86_64/xtask"],
    cmd = "pwd && mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xtask experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xtask $@",
    local = 1,
)

genrule(
    name = "install_xsimulation",
    srcs = ["xsimulation"],
    outs = ["x86_64/xsimulation"],
    cmd = "mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xsimulation experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xsimulation $@",
    local = 1,
)

genrule(
    name = "install_xopengl",
    srcs = ["xopengl"],
    outs = ["x86_64/xopengl"],
    cmd = "mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xopengl experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/pandaUser/xopengl $@",
    local = 1,
)

genrule(
    name = "install_xmotor",
    srcs = ["//experimental/users/sschaal/SL/panda:xpanda"],
    outs = ["x86_64/xmotor"],
    cmd = "mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/panda/xmotor experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/panda/xmotor $@",
    local = 1,
)

genrule(
    name = "install_xpest",
    srcs = ["//experimental/users/sschaal/SL/panda:xpest"],
    outs = ["x86_64/xpest"],
    cmd = "mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/panda/xpest experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/panda/xpest $@",
    local = 1,
)

genrule(
    name = "install_xpanda",
    srcs = ["//experimental/users/sschaal/SL/panda:xpanda"],
    outs = ["x86_64/xpanda"],
    cmd = "mkdir -p experimental/users/sschaal/SL/pandaUser/x86_64 && cp -f $(BINDIR)/experimental/users/sschaal/SL/panda/xpanda experimental/users/sschaal/SL/pandaUser/x86_64/ && cp  -f $(BINDIR)/experimental/users/sschaal/SL/panda/xpanda $@",
    local = 1,
)
