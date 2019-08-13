function stl2msh(name)
  model = createpde(3);
  importGeometry(model,[name,'.STL']);
  mesh=generateMesh(model,'GeometricOrder','linear', 'Hmax', 5.0);
  createMSHfile(mesh, name);
end