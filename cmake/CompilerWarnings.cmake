# Warnings-as-errors policy applied to every target we own (never to third-party code).
#   MSVC : /W4 /WX /permissive-
#   GCC/Clang : -Wall -Wextra -Wpedantic -Werror (plus a few useful extras)
function(amp_set_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /permissive- /utf-8)
    else()
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror
            -Wshadow -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
            -Woverloaded-virtual -Wconversion -Wsign-conversion -Wdouble-promotion)
        # -Wnull-dereference is deliberately omitted: at -O3 GCC reports false positives
        # inside inlined Eigen dynamic-vector code.
    endif()
endfunction()
