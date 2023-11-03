# basic source code CMakeLists.txt

# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/uboone/app/users/cerati/products/torch_scatter/


# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/uboone/app/users/cerati/products/torch_sparse/
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/uboone/app/users/cerati/products/libtorch
# export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/uboone/app/users/cerati/products/pytorch-install
# export PRODUCTS=$PRODUCTS:/uboone/app/users/cerati/products/ (this is probably doing nothing right now)

# unsetup libtorch

find_package(TorchScatter REQUIRED)
include_directories(SYSTEM /uboone/app/users/cerati/products/torch_scatter/include/)
link_directories(/uboone/app/users/cerati/products/torch_scatter/lib64)

# find_package(TorchSparse REQUIRED)
# include_directories(SYSTEM /uboone/app/users/cerati/products/torch_sparse/include/)
# link_directories(/uboone/app/users/cerati/products/torch_sparse/lib64)

#include_directories(SYSTEM /uboone/app/users/cerati/products/libtorch/include/)
#link_directories(/uboone/app/users/cerati/products/libtorch/lib)

include_directories(SYSTEM /uboone/app/users/cerati/products/pytorch-install/include/)
link_directories(/uboone/app/users/cerati/products/pytorch-install/lib)

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
    torch
    torchscatter
    #torchsparse
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
