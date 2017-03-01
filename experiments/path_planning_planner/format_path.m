function [ output ] = format_path( path )
%FORMAT_PATH Summary of this function goes here
%   Detailed explanation goes here
format bank
    time = posixtime(datetime('now'));
    poses = cell(size(path, 1), 1);
    for i = 1:size(path, 1)
        t = {};
        t.header.seq = i + 1;
        %t.header.stamp = int2str(time);
        t.header.frame_id = 'map';
        t.pose.position.x = path(i, 1);
        t.pose.position.y = path(i, 2);
        t.pose.position.z = 0;

        t.pose.orientation.x = 0;
        t.pose.orientation.y = 0;
        t.pose.orientation.z = 0;
        t.pose.orientation.w = 0;

        poses{i} = t;
    end

    output = {};
    output.header.seq = 1;
    %output.header.stamp = int2str(time);
    output.header.frame_id = 'map';
    output.poses = poses;
end

