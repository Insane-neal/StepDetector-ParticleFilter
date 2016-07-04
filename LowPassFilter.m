function[output] = LowPassFilter(lastOutput, input, ALPHA)
if isnan(lastOutput)
    output = input;
    return;
end
    output = ALPHA* lastOutput + (1-ALPHA)*input;
end
