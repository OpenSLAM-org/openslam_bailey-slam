function plot_feature_loci(data)
%function plot_feature_loci(data)
%
% From the offline stored data of the simulator, plot the 
% paths of the feature estimates over time.
%

hold on 
N= length(data.state);
maxf= length(data.state(N).x) - 3;
flocus= zeros(maxf,N);
iold= maxf;

for i = N:-1:1 
    ii= length(data.state(i).x)- 3;
    if ii>0, flocus(1:ii, i)= data.state(i).x(4:end); end
    
    if ii<iold
        for j=(ii+1):2:iold
            jj= (i+1):N;
            plot(flocus(j, jj), flocus(j+1, jj), 'k')
        end
        iold= ii;
    end    
end
