function stl2msh(name)
  model = createpde(3);
  importGeometry(model,[name,'.stl']);
  mesh=generateMesh(model,'GeometricOrder','linear', 'Hmax', 0.5);
  createMSHfile(mesh, name);
end