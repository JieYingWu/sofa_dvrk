function createMSHfile(mesh,filename)
%mesh file needs to be tetrahedron based, this is achieved by linear as the
%geometricOrder
% mesh=generateMesh(model,'GeometricOrder','linear','Hmax',2.5)

Nodes=[1:size(mesh.Nodes,2);mesh.Nodes];
% Elements=[1:size(mesh.Elements,2);mesh.Elements];
A(1:4,1:size(mesh.Elements,2))=1;
A(1,:)= A(1,:)*4;
A(4,:)= A(4,:)*4;

filename=sprintf('%s.msh',filename);
fileID=fopen(filename,'w');
fprintf(fileID,'%s \n','$Nodes');
fprintf(fileID,'%i \n',size(mesh.Nodes,2));
fprintf(fileID,'%i %f %f %f \n',Nodes);
fprintf(fileID,'%s \n','$EndNodes');
fprintf(fileID,'%s \n','$Elements');
fprintf(fileID,'%i \n',size(mesh.Elements,2));
fprintf(fileID,'%i %i %i %i %i %i %i %i %i \n',[1:size(mesh.Elements,2);A;mesh.Elements]);
% The use of 4 1 1 4 is not fully cleared up yet, but it has to do with the
% way the tetrahedrons are created
% fprintf(fileID,'%i %i %i %i %i \n',[1:size(mesh.Elements,2);mesh.Elements]);
fprintf(fileID,'%s \n','$EndElements');
fclose(fileID)

end