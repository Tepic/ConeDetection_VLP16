function xyz = readXYZ(obj)
            %readXYZ Returns the (x,y,z) coordinates of all points
            %   XYZ = readXYZ(OBJ) extracts the (x,y,z) coordinates from
            %   all points in the point cloud object OBJ and returns them
            %   as an Nx3 or HxWx3 matrix of 3D point
            %   coordinates. Each 3-vector represents one 3D point.
            %
            %   If the point cloud contains N points, the returned matrix
            %   has Nx3 elements (or a HxWx3 matrix if the PreserveStructure
            %   property is set to true).
            %
            %   If the point cloud does not contain the 'x', 'y' and 'z'
            %   fields, this function will display an error.
            %
            %
            %   Example:
            %
            %      % Retrieve 3D point coordinates
            %      xyz = readXYZ(pcloud);
            %
            %      % Plot the points with a 3D scatter plot
            %      scatter3(xyz(:,1), xyz(:,2), xyz(:,3), '.');
            
            data = obj.Data;
            
            try
                % Get field indices for X, Y, and Z coordinates
                xIndex = obj.getFieldNameIndex('x');
                yIndex = obj.getFieldNameIndex('y');
                zIndex = obj.getFieldNameIndex('z');
            catch ex
                newex = MException(message('robotics:ros:pointcloud:InvalidXYZData'));
                throw(newex.addCause(ex));
            end
            
            % Get byte index only once (this is expensive)
            byteIdx = obj.getByteIndexForField('x');
            
            % Calculate the byte offsets for the different fields
            % This helps with performance, since we can re-use the
            % already calculated byte index
            xOff = double(obj.Fields(xIndex).Offset);
            yOff = obj.relativeFieldOffset(xOff, yIndex);
            zOff = obj.relativeFieldOffset(xOff, zIndex);
            
            % Retrieve the XYZ data and concatenate into one matrix
            xyz = [obj.readFieldFromData('x', data, byteIdx, []), ...
                obj.readFieldFromData('y', data, byteIdx + yOff, []), ...
                obj.readFieldFromData('z', data, byteIdx + zOff, [])];
            
            % Reshape the output if requested by the user
            if obj.reshapeOutput
                xyz = reshape(xyz, obj.Width, obj.Height, 3);
                xyz = permute(xyz, [2 1 3]);
            end
        end