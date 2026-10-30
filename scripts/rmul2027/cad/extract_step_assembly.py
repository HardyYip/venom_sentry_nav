import argparse
import hashlib
import json
from pathlib import Path
import time

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString, TCollection_AsciiString
from OCP.TDF import TDF_LabelSequence, TDF_Label, TDF_Tool
from OCP.TDataStd import TDataStd_Name
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool, XCAFDoc_ColorTool
from OCP.TopLoc import TopLoc_Location
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Sphere, GeomAbs_Plane
from OCP.BRepTools import BRepTools


def entry(label):
    value = TCollection_AsciiString()
    TDF_Tool.Entry_s(label, value)
    return value.ToCString()


def name(label):
    value = TDataStd_Name()
    if label.FindAttribute(TDataStd_Name.GetID_s(), value):
        return value.Get().ToExtString().replace('\n', '')
    return entry(label)


def xyz(point):
    return [point.X(), point.Y(), point.Z()]


def bbox(shape):
    box = Bnd_Box()
    BRepBndLib.AddOptimal_s(shape, box, False, False)
    return None if box.IsVoid() else list(box.Get())


def features(shape):
    result = []
    iterator = TopExp_Explorer(shape, TopAbs_FACE)
    while iterator.More():
        face = TopoDS.Face_s(iterator.Current())
        surface = BRepAdaptor_Surface(face, True)
        props = GProp_GProps()
        BRepGProp.SurfaceProperties_s(face, props)
        record = dict(area_mm2=props.Mass())
        kind = surface.GetType()
        if kind == GeomAbs_Cylinder and surface.Cylinder().Radius() > 5:
            item = surface.Cylinder()
            record.update(type='cylinder', radius_mm=item.Radius(),
                          axis_origin_mm=xyz(item.Location()), axis=xyz(item.Axis().Direction()))
        elif kind == GeomAbs_Sphere and surface.Sphere().Radius() > 5:
            item = surface.Sphere()
            record.update(type='sphere', radius_mm=item.Radius(), center_mm=xyz(item.Location()))
        elif kind == GeomAbs_Plane and props.Mass() > 300:
            item = surface.Plane()
            record.update(type='plane', center_mm=xyz(props.CentreOfMass()),
                          normal=xyz(item.Axis().Direction()))
        else:
            iterator.Next()
            continue
        result.append(record)
        iterator.Next()
    return sorted(result, key=lambda item:item['area_mm2'], reverse=True)[:30]


def extract(source, output):
    output.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    print('Reading STEP', source, flush=True)
    status = reader.ReadFile(str(source))
    if status != IFSelect_RetDone:
        raise RuntimeError('STEP read failed: '+str(status))
    print('Transferring assembly', flush=True)
    doc = TDocStd_Document(TCollection_ExtendedString('cad-layout'))
    if not reader.Transfer(doc):
        raise RuntimeError('STEP transfer failed')
    tool = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    records = []
    def walk(instance, parent_location, path, parent_visible):
        target = TDF_Label()
        if not XCAFDoc_ShapeTool.GetReferredShape_s(instance, target):
            target = instance
        local = XCAFDoc_ShapeTool.GetLocation_s(instance)
        location = parent_location.Multiplied(local)
        transform = location.Transformation()
        label = name(target)
        shape = XCAFDoc_ShapeTool.GetShape_s(instance).Moved(parent_location)
        visible = parent_visible and XCAFDoc_ColorTool.IsVisible_s(instance) and XCAFDoc_ColorTool.IsVisible_s(target)
        box = bbox(shape)
        item = dict(instance=entry(instance),definition=entry(target),name=label,
                    path=path+[label],visible=visible,
                    assembly=XCAFDoc_ShapeTool.IsAssembly_s(target),
                    transform_cad=[[transform.Value(i,j) for j in range(1,5)] for i in range(1,4)],
                    bbox_mm=box)
        if box:
            item['extent_mm']=[box[i+3]-box[i] for i in range(3)]
            item['bbox_center_mm']=[(box[i+3]+box[i])/2 for i in range(3)]
        if 'MID-360' in label.upper():
            item['features_world']=features(shape)
            filename='mid-'+entry(instance).replace(':','-')+'.brep'
            BRepTools.Write_s(shape, str(output/filename))
            item['world_brep']=filename
        records.append(item)
        children=TDF_LabelSequence()
        XCAFDoc_ShapeTool.GetComponents_s(target, children, False)
        for i in range(1,children.Length()+1):
            walk(children.Value(i), location, item['path'], visible)
    for i in range(1, roots.Length()+1):
        walk(roots.Value(i), TopLoc_Location(), [], True)
    result=dict(source=str(source),bytes=source.stat().st_size,
                source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                root_count=roots.Length(),records=records,elapsed_wall_s=time.monotonic()-started,
                geometry_unit='millimetre (OCCT transfer default); verify known physical dimensions')
    (output/'assembly.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps(dict(roots=result['root_count'],records=len(records),elapsed=result['elapsed_wall_s'],
                         radars=[{k:r[k] for k in ['name','path','assembly','visible','bbox_mm','extent_mm']} for r in records if 'MID-360' in r['name'].upper()]),ensure_ascii=False),flush=True)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('source',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    extract(args.source.resolve(),args.output.resolve())
