function [ ] = TestRange( Parameter, Min, Max, Name )
%TESTRANGE This function takes an input 'paramater' and checks that it's within
%two limits ('Min' and 'Max'). It prints an error if not  

if Parameter <  Min || Parameter > Max 
    error('Input Paramater %s, value %d, is out of range', Name, Parameter)
end

end

