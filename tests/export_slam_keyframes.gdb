# Diagnostic export of SLAM's archived keyframe features, NOT raw bag scans.
# TCK_EXPORT_DIR must name the output directory. Run from the project root:
# TCK_EXPORT_DIR=run/.../keyframes gdb -batch -x tests/export_slam_keyframes.gdb --args bin/test_vilo_bag ...
set pagination off
set confirm off
set auto-solib-add off
set print thread-events off
handle SIGINT nostop noprint pass
break example/test_vilo_bag.cpp:266
run
python
import gdb,json,os
directory=os.environ['TCK_EXPORT_DIR']
os.makedirs(directory,exist_ok=True)
if gdb.newest_frame().name()!='main':
    gdb.execute('sharedlibrary libvilo')
    gdb.execute('thread apply all bt 10')
    raise RuntimeError('SLAM stopped before final trajectory export')
archive=gdb.newest_frame().read_var('archive')
vector=archive['states']['_M_impl']
begin=vector['_M_start'];end=vector['_M_finish']
inferior=gdb.selected_inferior()
def coefficients(value,n):
    return [float(value['m_storage']['m_data']['array'][i]) for i in range(n)]
rows=[]
for index in range(int(end-begin)):
    state=(begin+index).dereference()
    if not bool(state['key']) or int(state['features']['px'])==0:continue
    cloud=state['features']['px'].dereference()
    points=cloud['points']['_M_impl']
    first=points['_M_start'];last=points['_M_finish']
    # PointXYZI is 32 bytes in this project's 16-byte PCL ABI. With automatic
    # shared-library symbols disabled, GDB may expose an incomplete 8-byte type.
    stride=32
    count=(int(last)-int(first))//stride
    name='key_%06d.bin'%int(state['id'])
    with open(os.path.join(directory,name),'wb') as output:
        if count:output.write(bytes(inferior.read_memory(int(first),count*stride)))
    pose=state['Twl']
    rows.append({'index':index,'id':int(state['id']),'time':float(state['time']),
                 'count':count,'stride':stride,'file':name,
                 'translation':coefficients(pose['translation_'],3),
                 'quaternion':coefficients(pose['so3_']['unit_quaternion_']['m_coeffs'],4)})
with open(os.path.join(directory,'index.json'),'w') as output:json.dump(rows,output,indent=2)
print('DIAG exported actual SLAM keyframes',len(rows),'points',sum(r['count'] for r in rows))
end
disable 1
continue
quit
