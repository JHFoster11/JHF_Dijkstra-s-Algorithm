% Jacob Foster
% 2/20/2023
% CE-642
% Week 7 Assignment - Dijkstra’s Algorithm

% Reminder that position (1,1) is actually vertex # 0 in this assignment. This code can convert
% node #'s to 1...inf format and then outputs back to 0...inf format.
% (1,2) = distance from 0 to vertex #1 which equals 4
% (2,8) = distance from 1 to vertex #7 which equals 11

% This code also has extra features. One extra feature is checking
% for out of range source and destination nodes that the user inputs.
% The other feature is exit handling in case the user wants to exit the
% program after the program has been started. These features are mostly
% unnecessary for the Dijkstra’s Algorithm but have some utility.

% The cost matrix needs to be defined manually.

clc
clear all
close all
exit=0;

cost_matrix=[0 4 0 0 0 0 0 8 0; 4 0 8 0 0 0 0 11 0; 0 8 0 7 0 4 0 0 2; 0 0 7 0 9 14 0 0 0; 0 0 0 9 0 10 0 0 0; 0 0 4 14 10 0 2 0 0; 0 0 0 0 0 2 0 1 6; 8 11 0 0 0 0 1 0 7; 0 0 2 0 0 0 6 7 0];

% If your graphical representation of your network is in 0...inf format then
% this code will be able to handle user inputs of 0...inf since MATLAB is
% in 1...inf format. The outputs are even converted back to 0...inf format for easy analysis with your visualization. 
% If your graphical representation is in 1...inf then no
% actions/conversions are taken on the source and destination code.
format=input('Are your graphical nodes in the 0...inf format? (Y/N/EXIT): ','s'); 
format=lower(format);% EH
if (isempty(format))% EH
   exit=1;% EH
   fprintf('Exiting Program....... \n');% EH
elseif(all(format=='exit'))
   exit=1;% EH
   fprintf('Exiting Program....... \n');% EH
else
    if(format=='y')
        format=1;
    else
        format=0;
    end
end

while exit == 0  % This is just used to exit the program if the user enters nothing in the input prompt. EH = Exit Handling
    valid = false;
    while valid == false % Ensure that a valid node is selected. Node needs to be within cost matrix range.
       s=input('Enter the source node number (#/EXIT): ','s');
       s=lower(s); % EH
       if(isempty(s))% EH
           exit=1;% EH
           break % EH
       elseif(isempty(s) ||all(s~='exit'))% EH
           s=str2num(s);% EH
           if(format==1)
              s=s+1; % Convert node to 1..inf format
           end
           if(s <= size(cost_matrix,1))
               valid = true;
           else
               fprintf('Invalid node! That node is out of bounds of the cost matrix. Try again. \n');
           end
       else% EH
           exit=1;% EH
           break% EH
       end% EH
    end


    if(exit==1) % EH
        fprintf('Exiting Program....... \n');% EH
        break % EH
    end % EH

    
    loopall=input('Do you want to find shortest path to all nodes from source? (Y/N/EXIT)>','s');
    loopall=lower(loopall);% EH
    if (isempty(loopall))% EH
       exit=1;% EH
       fprintf('Exiting Program....... \n');% EH
       break% EH
    elseif(all(loopall=='exit'))
       exit=1;% EH
       fprintf('Exiting Program....... \n');% EH
       break% EH
    elseif(loopall=='n')
        valid=false;
        while valid == false % Ensure that a valid node is selected. Node needs to be within cost matrix range.
           d=input('Enter the destination node number (#/EXIT): ','s');
           d=lower(d);% EH
           if(isempty(d))% EH
               exit=1;% EH
               break % EH
           elseif(all(d~='exit'))% EH
               d=str2num(d);% EH
               if(format==1)
                  d=d+1; % Convert node to 1..inf format
               end
               if(d <= size(cost_matrix,1))
                   valid = true;
               else
                   fprintf('Invalid node! That node is out of bounds of the cost matrix. Try again. \n');
               end
           else% EH
               exit=1;% EH
               break  % EH 
           end% EH
        end
    end
    
    if(exit==1) % EH
        fprintf('Exiting Program....... \n');% EH
        break% EH
    end% EH
    
    all_sp=[];
    all_spcost=[];
    if(loopall=='y')
        for y = 1:size(cost_matrix,1)
            if(y~=s)
                [sp, spcost] = dijkstra_opt(cost_matrix, s, y, format);
                 all_sp{y} = sp;
                 all_spcost(y) = spcost;
            else
                 all_sp{y}=0;
                 all_spcost(y)=0;
            end
        end
    else
        [sp, spcost] = dijkstra_opt(cost_matrix, s, d, format);
    end
    fprintf('---------------------------------------\n');
    exit=1;% EH
end% EH


function [sp, spcost] = dijkstra_opt(cost_matrix, s, d,format)
    % This is an implementation of the dijkstra´s algorithm, wich finds the 
    % minimal cost path between two nodes. 
    
    % n: the number of nodes in the network;
    % s: source node index;
    % d: destination node index;


    %This implementatios is inspired by the Jorge Ignacio Barrera Alviar's 
    %implememtation of the dijkstra's algorithm, available at
    %http://www.mathworks.com/matlabcentral/fileexchange
    %file ID 14661
    
    % Dijkstra's Algorithm also referenced by:
    % LaValle, S. (2006). Planning algorithms: Chapter 2: Discrete planning (pp. 27-72). Planning algorithms. Cambridge University Press. http://lavalle.pl/planning/ch2.pdf

    %Author: Jacob Foster. 02/23/2023


    n=size(cost_matrix,1);
    S(1:n) = 0;     %s, vector, set of visited vectors
    dist(1:n) = inf;   % it stores the shortest distance between the source node and any other node;
    prev(1:n) = inf;    % Previous node, informs about the best previous node known to reach each network node 
    init = 1;
    dist(s) = 0;


    while sum(S)~=n % If some nodes are still alive, keep searching
        candidate=[];
        for i=1:n
            if S(i)==0
                candidate=[candidate dist(i)];
            else
                candidate=[candidate inf];
            end
        end

        [u_index u]=min(candidate);
        S(u)=1;
        if(u==d) %If the min canidate equals the destination then search is over! Stop searching for faster computation.
            S(1:n) = 1;
        else
            for i=1:n
                if(dist(u)+cost_matrix(u,i))<dist(i) && cost_matrix(u,i) ~= 0 % If the cost_matrix being evaluated is 0 then it is not connected to the current source node. Leave the dist as inf.
                    dist(i)=dist(u)+cost_matrix(u,i);
                    prev(i)=u; % Update the previously vistied node for later use in back calculation of path
                end
            end
        end
    end


    sp = [d];

    while sp(1) ~= s
        if prev(sp(1))<=n
            sp=[prev(sp(1)) sp];
        else
            error;
        end
    end;
    spcost = dist(d); % The total distance has been added each iteration if the path continues. Take the total cost from the destination node
    if(format==1)
        sp = sp - 1; % Convert path array back to 0..inf format
    end
    
    % Print Results!
    if(format==1)
        fprintf('---------------------------------------\n');
        fprintf('The optimal path from node %d to node %d is: \n',s-1,d-1);
        fprintf('%d \n',sp);
        fprintf('The cost of moving from node %d to node %d is: \n',s-1,d-1);
        fprintf('%d \n',spcost);
    else
        fprintf('---------------------------------------\n');
        fprintf('The optimal path from node %d to node %d is: \n',s,d);
        fprintf('%d \n',sp);
        fprintf('The cost of moving from node %d to node %d is: \n',s,d);
        fprintf('%d \n',spcost);
    end
end