function [expr_repl] = sym_replace(expr, vars, vals)
%REPL_SYM
%   Replaces Vars with Vals in Expr, reutrns N
    expr_repl = expr;
    for i = 1:length(vals)
        expr_repl = subs(expr_repl, vars(i), vals(i));
    end
end

