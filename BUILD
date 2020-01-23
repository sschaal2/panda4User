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
        "@LAB_ROOT//SL:SLcommon",
        "@LAB_ROOT//SL:SLtask",
        "@LAB_ROOT//panda",
        "@LAB_ROOT//panda:panda_task",
        "@LAB_ROOT//panda:xmotor",
        "@LAB_ROOT//panda:xpanda",
        "@LAB_ROOT//panda:xpest",
        "@LAB_ROOT//utilities:utility",
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
        "@LAB_ROOT//SL:SLcommon",
        "@LAB_ROOT//SL:SLopenGL",
        "@LAB_ROOT//panda",
        "@LAB_ROOT//panda:panda_openGL",
        "@LAB_ROOT//utilities:utility",
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
        "@LAB_ROOT//SL:SLcommon",
        "@LAB_ROOT//SL:SLsimulation",
        "@LAB_ROOT//panda",
        "@LAB_ROOT//panda:panda_simulation",
        "@LAB_ROOT//utilities:utility",
    ],
)

genrule(
    name = "install_xtask",
    srcs = ["xtask"],
    outs = ["x86_64mac/xtask"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/pandaUser/xtask pandaUser/x86_64mac/ && cp  -f $(BINDIR)/pandaUser/xtask $@",
    local = 1,
)

genrule(
    name = "install_xsimulation",
    srcs = ["xsimulation"],
    outs = ["x86_64mac/xsimulation"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/pandaUser/xsimulation pandaUser/x86_64mac/ && cp  -f $(BINDIR)/pandaUser/xsimulation $@",
    local = 1,
)

genrule(
    name = "install_xopengl",
    srcs = ["xopengl"],
    outs = ["x86_64mac/xopengl"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/pandaUser/xopengl pandaUser/x86_64mac/ && cp  -f $(BINDIR)/pandaUser/xopengl $@",
    local = 1,
)

genrule(
    name = "install_xmotor",
    srcs = ["@LAB_ROOT//panda:xpanda"],
    outs = ["x86_64mac/xmotor"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/external/LAB_ROOT/panda/xmotor pandaUser/x86_64mac/ && cp  -f $(BINDIR)/external/LAB_ROOT/panda/xmotor $@",
    local = 1,
)

genrule(
    name = "install_xpest",
    srcs = ["@LAB_ROOT//panda:xpest"],
    outs = ["x86_64mac/xpest"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/external/LAB_ROOT/panda/xpest pandaUser/x86_64mac/ && cp  -f $(BINDIR)/external/LAB_ROOT/panda/xpest $@",
    local = 1,
)

genrule(
    name = "install_xpanda",
    srcs = ["@LAB_ROOT//panda:xpanda"],
    outs = ["x86_64mac/xpanda"],
    cmd = "mkdir -p pandaUser/x86_64mac && cp -f $(BINDIR)/external/LAB_ROOT/panda/xpanda pandaUser/x86_64mac/ && cp  -f $(BINDIR)/external/LAB_ROOT/panda/xpanda $@",
    local = 1,
)
