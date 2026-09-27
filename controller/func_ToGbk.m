function did = func_ToGbk(p)
%FUNC_TOGBK  把一个 .m 转成 GBK 并去掉 UTF-8 BOM; 返回是否真的转过
%
%   为什么要转: 部署/MIL 机器是**中文 Windows 上的 R2018a**。MATLAB 从 R2020a
%   起才默认按 UTF-8 读 .m, 之前一律按系统 ANSI 代码页(中文 Windows = GBK/CP936)。
%   于是 UTF-8 的中文注释会乱码; 带 UTF-8 BOM 的文件更是直接报错, 且报在**行1列1**:
%     "文本字符无效。请检查不受支持的符号、不可见的字符或非 ASCII 字符的粘贴。"
%   (实测 func_VehicleParams.m 就是这样挂的。)
%
%   判 UTF-8 的办法: 按 UTF-8 解一遍再编回去, 字节完全一样才算合法 UTF-8。
%   这样已经是 GBK 的文件不会被误判(GBK 的中文字节序列一般不是合法 UTF-8)。
%
%   make_deploy(HIL 包) 和 make_mil(MIL 包) 共用本函数。
did = 0;
f = fopen(p,'r'); b = fread(f, inf, '*uint8').'; fclose(f);

bom = uint8([239 187 191]);
hasBom = numel(b) >= 3 && isequal(b(1:3), bom);
if hasBom, b = b(4:end); end

t = native2unicode(b, 'UTF-8');
isUtf8 = isequal(uint8(unicode2native(t,'UTF-8')), b);
if ~isUtf8
    if hasBom                       % 极少见: 有 BOM 但内容是 GBK
        f = fopen(p,'w'); fwrite(f, b); fclose(f);
        did = 1;
    end
    return                          % 已经是 GBK, 不动
end

%  GBK(CP936) 装不下的几个字符, 先换成 ASCII 等价物。
%  实测只有这三个: U+00B2 上标2 / U+26A0 警告三角 / U+FE0F 变体选择符。
t = strrep(t, char(9888), '!!');    % U+26A0
t = strrep(t, char(65039), '');     % U+FE0F
t = strrep(t, char(178), '^2');     % U+00B2

g = unicode2native(t, 'GBK');
%  回读校验: 转回来必须和转换前一致, 否则说明有字符被吃成了 '?'
back = native2unicode(g, 'GBK');
if ~strcmp(back, t)
    n = find(back(1:min(numel(back),numel(t))) ~= t(1:min(numel(back),numel(t))), 1);
    error('func_ToGbk:Loss', ...
          '%s 转 GBK 有字符丢失(首个在第 %d 个字符附近), 请在本函数里加替换规则', p, n);
end
f = fopen(p,'w'); fwrite(f, g); fclose(f);
did = 1;
end
