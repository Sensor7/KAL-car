function s = calculate_parameter_s(x, y)
    % 确保输入的x和y是列向量
    x = x(:);
    y = y(:);

    % 计算每个点到前一个点的距离，并累加得到折线长度
    n = length(x);
    s = zeros(n, 1);
    for i = 2:n
        s(i) = s(i-1) + sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2);
    end
end