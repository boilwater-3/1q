# 源文件行读取辅助：file(STRINGS) + 首行 UTF-8 BOM 剥离。
#
# 背景：仓库 C/C++ 源文件统一携带 UTF-8 BOM（VS2015 集成端按系统代码页
# 误读无 BOM 文件，中文注释错位截断；cl.exe 对带 BOM 文件无条件按 UTF-8
# 解码）。CMake 的 file(READ)/file(STRINGS) 不感知 BOM：BOM 字节粘在首行
# 行首，会使 ^ 锚定的行首正则（include 风格/方向/分层守护）漏检首行，
# 或使首行注释跳过逻辑失效（误报）。凡按行扫描源文件的守护统一经本函数读取。

function(oneq_read_source_lines out_var source_file)
    file(STRINGS "${source_file}" _lines)
    list(LENGTH _lines _line_count)
    if(_line_count GREATER 0)
        string(ASCII 239 187 191 _bom)
        list(GET _lines 0 _first_line)
        string(FIND "${_first_line}" "${_bom}" _bom_pos)
        if(_bom_pos EQUAL 0)
            string(SUBSTRING "${_first_line}" 3 -1 _first_line)
            list(REPLACE_AT _lines 0 "${_first_line}")
        endif()
    endif()
    set(${out_var} "${_lines}" PARENT_SCOPE)
endfunction()
