set(CUSTOM_DEPS
    OpenMRNIDF
    OpenMRNExtensions
    Utils
)

idf_component_register(SRCS TrainManager.cpp LazyInitTrainNode.cpp TrainFDISpace.cpp TrainPipHandler.cpp TrainSnipHandler.cpp PersistentTrainConfigSpace.cpp TrainCDISpace.cpp TrainFDISpace.cpp TrainIdentifyHandler.cpp FdiXmlGenerator.cpp XmlGenerator.cpp
                       INCLUDE_DIRS include
                       PRIV_INCLUDE_DIRS private_include
                       REQUIRES "${CUSTOM_DEPS}")
