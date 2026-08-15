# defaultのCMakeLists.txtを持たないproject用に自動生成されたfile。

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources} REQUIRES esp_driver_gptimer)

target_compile_options(
    ${COMPONENT_LIB}
    PRIVATE
    "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>"
)

if(AVI_99L_CHARACTERIZATION_BUILD)
    # ESP-IDF framework全体へ-O3を適用すると、framework内部の-Werrorと
    # GCCの強いarray-bounds解析が衝突し得る。timing qualificationで速度が必要な
    # MissionBoard自前componentだけを-O3にし、frameworkは公式performance(-O2)に保つ。
    target_compile_options(${COMPONENT_LIB} PRIVATE -O3)
endif()
