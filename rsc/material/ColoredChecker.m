function board = ColoredChecker(numSq, sqSz, c1, c2)
    % board = ColoredChecker(numSq, sqSz, c1, c2)
    % 
    % The purpose of this function is to create an image with a
    % checkerboard pattern. The number of squares, size of the squares, and
    % color of the squares can be customized.
    %
    % Inputs:
    %   numSq: The number of squares (min of 2, same for X and Y direction)
    %   sqSz:  The size of the squares in pixels.
    %   c1:    Color expressed as a string ('r','g','b','k') or RGB code
    %           ([R,G,B], where R,G,B \in [0,1]). If c2 is passed in, c1
    %           must be an RGB code. (optional)
    %   c2:    Color expressed as an RGB code. (optional)
    %
    % Outputs:
    %   board: An (N x N x 3) matrix with the image data. This data can be
    %           plotted using imshow(board), or saved with
    %           imwrite(board,'name.png').
    %   
    % Call Examples:
    %   ColoredChecker(numSq, sqSz) <-- all default colors (white + green)
    %   ColoredChecker(numSq, sqSz, ' ') <-- default white + pre-made color of choice ('r','g','b','k')
    %   ColoredChecker(numSq, sqSz, [R,G,B]) <-- default white, and an RGB
    %           code for the second color
    %   ColoredChecker(numSq, sqSz, [R,G,B], [R,G,B]) <-- use different RGB
    %           code for each color
    %
    % WARNING: Making both numSq and sqSz large will result in a very large
    %       image and can cause crashes. 
    %
    % NOTE: For same size as RaiSimOgre default checker_green, use 
    %       numSq = 2 and sqSz = 756/2
    %
    
    defaultWhite = [240,240,240]/255;
    if numSq<2
        numSq = 2;
    end 
    if nargin == 2
        c1 = defaultWhite;
        c2 = [  0, 160,   0]/255; 
    end
    if nargin == 3 && ischar(c1)
        if strcmpi('red',c1) || strcmpi('r',c1)
            c1 = defaultWhite;
            c2 = [200,60,60]/255;
        elseif strcmpi('blue',c1) || strcmpi('b',c1)
            c1 = defaultWhite;
            c2 = [0,180,240]/255;
        elseif strcmpi('black',c1) || strcmpi('k',c1)
            c1 = defaultWhite;
            c2 = [0,0,0]/255;
        else % Use green by default
            c1 = defaultWhite;
            c2 = [95,200,155]/255;
        end
    elseif nargin == 3 && isnumeric(c1) && length(c1) == 3
        c2 = c1;
        c1 = defaultWhite;
    end
    
    % Make base board
    base = {zeros(sqSz,sqSz),ones(sqSz,sqSz)};
    order = ones(1, numSq);
    order(1:2:numSq) = 0;
    order = repmat(order,numSq,1);
    order(1:2:numSq,:) = ~order(1:2:numSq,:);
    
    % Make board
    board = cell2mat(reshape(base(order+1),numSq,numSq));
    board = repmat(board,1,1,3);
    for n = 1:3
        sq1 = false(size(board));
        sq1(:,:,n) = board(:,:,n)==1;
        sq2 = false(size(board));
        sq2(:,:,n) = board(:,:,n)==0;
        
        board(sq1) = c1(n);
        board(sq2) = c2(n);
    end

    % Plots the results:
%     imshow(board)
%     f = gcf;
%     f.Units = 'inches';
%     f.Position = [4.5,1.5,8,8];
%     ax = f.Children(1);
%     ax.Units = 'inches';
%     ax.Position = [0.75,0.75,6.75,6.75];
%     set(ax,'LooseInset',get(ax,'TightInset'));

    % Saves the results to an image
%     imwrite(board,'checker_green.png'); % Use this to save image
end



