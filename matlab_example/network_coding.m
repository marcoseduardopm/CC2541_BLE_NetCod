clc; clear; close all;

%% Configuration Parameters
type = "BNC"; % Options: "DAF", "BNC", "DNC"
nodes = 3; % Number of transmitting nodes (2 or 3)
k1 = 1; % Messages per node in diffusion phase (1 or 2)
k2 = 2; % Messages per node in cooperation phase (1 or 2)
% Note: Not all combinations of type, nodes, k1, and k2 are defined.

M = nodes * k1; % Total number of messages in the system

%% Define Encoding Matrix Based on Parameters
% The encoding matrix consists of:
% - An identity matrix representing individual user messages.
% - A parity matrix defining how messages are combined in the network.

if k1 == 1 && k2 == 1
    % Case: 1 message per node in diffusion & cooperation phases (only for 2 nodes)
    if nodes == 2
        iden = [1 0; 0 1]; % Identity matrix (diffusion phase)

        % Define parity matrix based on selected type
        switch type
            case "DAF"
                par = [0 1; 
                       1 0]; % Direct Amplify and Forward
            case "BNC"
                par = [1 1; 
                       1 1]; % Binary Network Coding
            case "DNC"
                par = [1 1; 
                       1 2]; % Deterministic Network Coding
        end
    end

elseif k1 == 1 && k2 == 2
    % Case: 1 message per node in diffusion, 2 in cooperation (only for 3 nodes)
    if nodes == 3
        iden = eye(3); % 3x3 identity matrix

        switch type
            case "DAF"
                par = [0 0 1 0 1 0;
                       1 0 0 0 0 1;
                       0 1 0 1 0 0];
            case "BNC"
                par = ones(3, 6); % All ones matrix
            case "DNC"
                par = [1 1 2 3  4  7;
                       1 2 3 13 11 5;
                       5 4 1 1  15 14];
        end
    end
end
% Construct the full encoding matrix (identity + parity)
encoding_matrix = [iden par]

%% Generate Random Messages
I = randi([0 255], M, 1); % Original messages from users

%% First Consideration: Outage Between Users (Inter-user Outage)
% If a user cannot decode another user's message during the diffusion phase, 
% we must zero out the corresponding entry in the encoding matrix.

% Example: If user 2 cannot receive user 1's message, update the matrix:
% Original:
%   1  0  1  1
%   0  1  1  2
%
% After outage:
%   1  0  1  0  <- Zeroed (user 1’s contribution removed from user 2’s encoded message)
%   0  1  1  2

encoding_matrix(1, end) = 0; % Simulate an inter-user outage scenario

%% Encode Messages at Nodes
encoded_messages = I' * encoding_matrix; % Generate encoded messages

%% Second Consideration: Outage Between User and Destination
% If a message is lost during transmission to the destination, it is zeroed out 
% instead of being removed.

% Example: Assume the destination did not receive the last cooperation-phase message
lost_indices = [4]; % Indices of lost messages

% Apply zeroing to lost messages
received_messages = encoded_messages;
received_messages(lost_indices) = 0; % Zero out lost messages
encoding_matrix(:, lost_indices) = 0; % Zero corresponding encoding matrix columns

%% Decoding at Destination
% Use pseudo-inverse to estimate original messages
solution = received_messages * pinv(encoding_matrix); 

%% Display Results
disp(['Original Messages: ', num2str(I')]);
disp(['Encoded Messages: ', num2str(encoded_messages)]);
disp(['Lost Messages Indices: ', num2str(lost_indices)]);
disp(['Decoded Messages: ', num2str(round(solution))]);
