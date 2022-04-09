"""
    timeout(seconds, expr)

If `expr` does not complete with the specified timeframe `seconds`, send an interrupt exception.

# Examples
    @timeout 600 expensive_algorithm(input)
"""
macro timeout(seconds, expr)
    quote
        tsk = @task $expr
        schedule(tsk)
        Timer($seconds) do timer
            istaskdone(tsk) || Base.throwto(tsk, InterruptException())
        end
        fetch(tsk)
    end
end