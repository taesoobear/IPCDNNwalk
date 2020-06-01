package.resourcePath="../Resource/motion/"
package.scriptPath="../Resource/scripts/modules/"

package.path=package.path..";./?.lua" --;"..package.path
package.path=package.path..";../lua/?.lua"
package.path=package.path..";"..package.scriptPath.."?.lua"
package.path=package.path..";"..package.scriptPath.."/RigidBodyWin/?.lua"
package.path=package.path..";"..package.scriptPath.."/RigidBodyWin/subRoutines/?.lua"
package.path=package.path..";../MainLib/WrapperLua/?.lua"
package.path=package.path..";../src/taesooLib/MainLib/WrapperLua/?.lua"

require('mylib')
require('module')
