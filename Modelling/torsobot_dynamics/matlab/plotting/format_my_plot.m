function format_my_plot(fig_width, fig_height)
    % FORMAT_MY_PLOT Automatically upgrades all subplots in the current 
    % MATLAB figure to publication-quality standards, including yyaxis, 
    % sgtitle support, and figure sizing.
    %
    % Usage:
    %   format_my_plot()         % Uses default 6.5 x 5 inch size
    %   format_my_plot(6.5, 7)   % Sets figure to 6.5" wide by 7" tall

    % --- 0. Handle Inputs and Figure Sizing ---
    if nargin < 2
        fig_height = 5; % Default height in inches
    end
    if nargin < 1
        fig_width = 6.5; % Default width in inches
    end

    fig = gcf;
    
    % Set on-screen display size
    fig.Units = 'inches';
    current_pos = fig.Position;
    % Keep the figure's current bottom-left anchor, just update width/height
    fig.Position = [current_pos(1), current_pos(2), fig_width, fig_height];
    
    % Set export/printing size (Crucial for publication quality saving)
    fig.PaperUnits = 'inches';
    fig.PaperPositionMode = 'manual';
    fig.PaperPosition = [0, 0, fig_width, fig_height];
    fig.PaperSize = [fig_width, fig_height];

    % --- 1. Format the Super Title (sgtitle) ---
    % MATLAB tags the sgtitle text object specifically as 'suptitle'
    sgt = findall(fig, 'Type', 'Text', 'Tag', 'suptitle');
    if ~isempty(sgt)
        sgt.FontSize = 14;
        sgt.FontWeight = 'bold';
        sgt.FontName = 'Helvetica';
    end
    
    % --- 2. Find ALL axes in the current figure ---
    all_axes = findall(fig, 'Type', 'axes');
    
    % Loop through every subplot and apply formatting
    for i = 1:length(all_axes)
        ax = all_axes(i);
        
        % General Axes Formatting
        ax.FontSize = 12;          
        ax.FontName = 'Helvetica'; 
        ax.LineWidth = 1.2;        
        ax.Box = 'on';             
        ax.XColor = 'k';           % Force X-axis to black
        
        % Grid Formatting
        ax.GridAlpha = 0.15;       
        
        % Format Standard Title and X-Label
        if ~isempty(ax.Title.String)
            ax.Title.FontSize = 14;
            ax.Title.FontWeight = 'bold';
        end
        if ~isempty(ax.XLabel.String)
            ax.XLabel.FontSize = 12;
            ax.XLabel.FontWeight = 'bold';
        end
        
        % Format Y-Labels and YYAXIS Colors
        if length(ax.YAxis) == 1
            % STANDARD SINGLE AXIS
            ax.YColor = 'k'; % Force Y-axis to black
            if ~isempty(ax.YLabel.String)
                ax.YLabel.FontSize = 12;
                ax.YLabel.FontWeight = 'bold';
            end
        else
            % DUAL YYAXIS
            % Format Left Axis
            ax.YAxis(1).Color = 'k'; % Override default blue
            if ~isempty(ax.YAxis(1).Label.String)
                ax.YAxis(1).Label.FontSize = 12;
                ax.YAxis(1).Label.FontWeight = 'bold';
            end
            
            % Format Right Axis
            ax.YAxis(2).Color = 'k'; % Override default orange
            if ~isempty(ax.YAxis(2).Label.String)
                ax.YAxis(2).Label.FontSize = 12;
                ax.YAxis(2).Label.FontWeight = 'bold';
            end
        end
    end
    
    % --- 3. Find ALL legends and format them ---
    all_legends = findall(fig, 'Type', 'Legend');
    for i = 1:length(all_legends)
        leg = all_legends(i);
        leg.FontSize = 11;
        
        % The Professional Legend Background:
        leg.Box = 'on';                    
        leg.Color = [1 1 1];               
        leg.EdgeColor = [0.8 0.8 0.8];    
        % leg.Location = 'best';
    end
end