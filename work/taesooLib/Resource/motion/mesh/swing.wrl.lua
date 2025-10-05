-- this is actually a lua function that returns a table.
-- so, you can procedurally generate a model instead (for example, see tire.wrl.lua).
return {
	name='swing',
	body={
			name='pend1',
			jointType="rotate",
			jointAxis="X",
			translation =vector3(0.000000,2.300000,0.000000 ), -- joint의 위치
			geometry ={ 
				{
					'Box',
					translation=vector3(0.35,-0.5,0),
					size=vector3(0.1,1,0.1),
				},
				{
					'Box',
					translation=vector3(-0.35,-0.5,0),
					size=vector3(0.1,1,0.1),
				},
				{
					'Box',
					translation=vector3(0,-0.75,0),
					size=vector3(0.7,0.1,0.1),
				},
			},
			children={
				{
					name='pend2',
					jointType="rotate",
					jointAxis="X",
					translation =vector3(0.000000,-1.000000,0.000000 ), -- joint의 위치
					geometry ={ 
						{
							'Box',
							translation=vector3(0.35,-0.5,0),
							size=vector3(0.1,1,0.1),
						},
						{
							'Box',
							translation=vector3(-0.35,-0.5,0),
							size=vector3(0.1,1,0.1),
						},
						{
							'Box',
							translation=vector3(0,-1,0),
							size=vector3(0.7,0.1,0.1),
						},
					},
				}
			}
	}
}
