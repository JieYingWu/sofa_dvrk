function stl2msh(name)
  model = createpde(3);
  importGeometry(model,[name,'.stl']);
  mesh=generateMesh(model,'GeometricOrder','linear', 'Hmax', 1.0);
  createMSHfile(mesh, name);
end