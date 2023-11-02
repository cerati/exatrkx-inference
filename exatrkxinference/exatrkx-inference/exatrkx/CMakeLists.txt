# basic source code CMakeLists.txt

simple_plugin(TestInference "module"
    ${ART_FRAMEWORK_PRINCIPAL}
    ${ART_FRAMEWORK_SERVICES_REGISTRY}
    ${ART_FRAMEWORK_SERVICES_BASIC}
    ${ART_FRAMEWORK_SERVICES_OPTIONAL}
    ${ART_PERSISTENCY_PROVENANCE}
    ${ART_ROOT_IO_TFILE_SUPPORT}
    ${ROOT_CORE}
    ${ART_ROOT_IO_TFILESERVICE_SERVICE}
    ${ART_PERSISTENCY_COMMON}
    canvas
    ${ART_UTILITIES}
    ${MF_MESSAGELOGGER}
    ${FHICLCPP}
    cetlib
    cetlib_except
    ${ROOT_BASIC_LIB_LIST}
    lardataobj_RecoBase
)

simple_plugin(PandoraNuSliceHitsProducer "module"
    ${ART_FRAMEWORK_PRINCIPAL}
    ${ART_FRAMEWORK_SERVICES_REGISTRY}
    ${ART_FRAMEWORK_SERVICES_BASIC}
    ${ART_FRAMEWORK_SERVICES_OPTIONAL}
    ${ART_PERSISTENCY_PROVENANCE}
    ${ART_ROOT_IO_TFILE_SUPPORT}
    ${ROOT_CORE}
    ${ART_ROOT_IO_TFILESERVICE_SERVICE}
    ${ART_PERSISTENCY_COMMON}
    canvas
    ${ART_UTILITIES}
    ${MF_MESSAGELOGGER}
    ${FHICLCPP}
    cetlib
    cetlib_except
    ${ROOT_BASIC_LIB_LIST}
    nusimdata_SimulationBase
    lardataobj_RecoBase
)

install_headers()
install_source()
install_fhicl()
