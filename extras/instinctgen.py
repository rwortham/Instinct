#  Instinct Plan Export Utility for Dia
#  Copyright (c) 2015  Robert H. Wortham <r.h.wortham@gmail.com>
#
#  Modified from PyDia Code Generation from UML Diagram
#  Copyright (c) 2005  Hans Breuer <hans@breuer.org>

#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.

import sys, dia, datetime

class InstinctNode :
	def __init__ (self, name) :
		self.name = name
		self.attributes = {}
		self.comment = ""
		self.parents = []
		self.children = []
		self.node_type = ""
		self.node_id = 0		
		
	def AddAttribute(self, name, value, comment) :
		# print name
		self.attributes[name] = (name, value, comment)
		
	def SetComment(self, s) :
		self.comment = s
		
	def AddPar(self, par) :
		# print "AddPar", par, "\n"
		self.parents.append(par)
		
	def AddChild(self, child) :
		# print "AddChild", child, "\n"
		self.children.append(child)
		
	def SetNodeType(self, node_type) :
		self.node_type = node_type
		
	def SetNodeID(self, id) :
		self.node_id = id

class ObjRenderer :
	"Implements the Object Renderer Interface and transforms diagram into its internal representation"
	def __init__ (self) :
		# empty dictionary of nodes, actions, senses
		self.nodes = {}
		self.actions = {}
		self.senses = {}
		self.filename = ""
		
	def begin_render (self, data, filename) :
		self.filename = filename
		# not only reset the filename but also the other state, otherwise we would accumulate information through every export
		self.nodes = {}
		self.actions = {}
		self.senses = {}
		for layer in data.layers :
			# for the moment ignore layer info. But we could use this to spread accross different files
			for o in layer.objects :
				# print o.type.name, o.properties
				if o.type.name == "UML - Class" :
					# print o.properties["name"].value
					n = InstinctNode (o.properties["name"].value)
					n.SetComment(o.properties["comment"].value)
					# print o.properties["stereotype"].value
					n.SetNodeType(o.properties["stereotype"].value)
					# print o.properties["attributes"].value
					for attr in o.properties["attributes"].value :
						# see objects/UML/umlattributes.c:umlattribute_props
						# print "\t", attr[0], attr[2], attr[3]
						# name, type, value, comment, visibility, abstract, class_scope
						n.AddAttribute(attr[0], attr[2], attr[3])
					if n.node_type == "RobotSense" :
						n.SetNodeID(len(self.senses)+1)
						if o.properties["name"].value in self.senses.keys() :
							print "Duplicate RobotSense name:", o.properties["name"].value
						self.senses[o.properties["name"].value] = n
					elif n.node_type == "RobotAction" :
						n.SetNodeID(len(self.actions)+1)
						if o.properties["name"].value in self.actions.keys() :
							print "Duplicate RobotAction name:", o.properties["name"].value
						self.actions[o.properties["name"].value] = n
					else :
						n.SetNodeID(len(self.nodes)+1)
						if o.properties["name"].value in self.nodes.keys() :
							print "Duplicate node name:", o.properties["name"].value
						self.nodes[o.properties["name"].value] = n
		
		edges = {}
		for layer in data.layers :
			for o in layer.objects :
				for c in o.connections:
					for n in c.connected:
						if n.type.name == "UML - Association" :
							if str(n) in edges:
								continue
							edges[str(n)] = None
							if not (n.handles[0].connected_to and n.handles[1].connected_to):
								continue
							par = n.handles[0].connected_to.object
							chi = n.handles[1].connected_to.object
							if not par.type.name == "UML - Class" and chi.type.name == "UML - Class":
								continue
							par_name = par.properties["name"].value
							chi_name = chi.properties["name"].value
							# print par_name, chi_name
							self.nodes[chi_name].AddPar(par_name)
							self.nodes[par_name].AddChild(chi_name)
					
	def end_render(self) :
		# without this we would accumulate info from every pass
		self.nodes = {}

class InstinctRenderer(ObjRenderer) :
	def __init__(self) :
		ObjRenderer.__init__(self)
		
	def end_render(self) :
		f = open(self.filename, "w")
		f.write("// *** Instinct Robot Plan generated by dia/instinctgen.py ***\n")
		f.write("// *** %s %s\n\n" % (datetime.datetime.strftime(datetime.datetime.now(), '%Y-%m-%d %H:%M:%S'), self.filename))
		
		f.write("// *** First, clear the old plan and initialise the buffers ***\n")
		f.write("PLAN R C\n")
		# COUNT_ACTIONPATTERN COUNT_ACTIONPATTERNELEMENT COUNT_COMPETENCE COUNT_COMPETENCEELEMENT COUNT_DRIVE COUNT_ACTION
		action_patterns = 0
		action_pattern_elements = 0
		competences = 0
		competence_elements = 0
		drives = 0
		actions = 0
		for sk in self.nodes.keys() :
			node_type = self.nodes[sk].node_type
			if node_type == "ActionPattern" :
				action_patterns = action_patterns + 1
			elif node_type == "ActionPatternElement" :
				action_pattern_elements = action_pattern_elements + 1
			elif node_type == "Competence" :
				competences = competences + 1
			elif node_type == "CompetenceElement" :
				competence_elements = competence_elements + 1
			elif node_type == "Drive" :
				drives = drives + 1
			elif node_type == "Action" :
				actions = actions + 1
		f.write("// AP=%d, APE=%d, C=%d, CE=%d, D=%d, A=%d\n" % (action_patterns, action_pattern_elements, competences, competence_elements, drives, actions))
		f.write("PLAN R I %d %d %d %d %d %d\n\n" % (action_patterns, action_pattern_elements, competences, competence_elements, drives, actions))
			
		f.write("// *** These are the Plan Elements. ***\n\n")
		
		for sk in self.nodes.keys() :
			n = self.nodes[sk]
			if len(n.comment) > 0 :
				f.write ("// " + n.comment + "\n")
			f.write("// %s: %s\n" % (n.node_type, n.name))
			if len(n.parents) > 0 :
				f.write ("// \tParents: %s\n" % ", ".join(n.parents))
			if len(n.children) > 0 :
				f.write ("// \tChildren: %s\n" % ", ".join(n.children))

			self.render_node(n, f)
			f.write ("\n")

		f.write ("// *** Plan Element Names follow ... ***\n\n")
		
		for sn in self.nodes.keys() :
			n = self.nodes[sn]
			if len(n.comment) > 0 :
				f.write ("// " + n.comment + "\n")
			f.write("// %s: %s\n" % (n.node_type, n.name))
			f.write("PELEM %s=%s\n\n" % (n.name, n.node_id))

		f.write ("// *** Plan output complete. RobotSenses and RobotActions follow ... ***\n\n")
		
		for ss in self.senses.keys() :
			n = self.senses[ss]
			if len(n.comment) > 0 :
				f.write ("// " + n.comment + "\n")
			f.write("// %s: %s\n" % (n.node_type, n.name))
			if "SenseID" in n.attributes.keys() :
				attr = n.attributes["SenseID"]
				f.write("//\t%s=%s" % (attr[0], attr[1]))
				if attr[2] != "" :
					f.write("\t// %s" % attr[2])
				f.write("\n")
				f.write("RSENSE %s=%s\n" % (n.name, attr[1]))
			f.write("\n")

		for sa in self.actions.keys() :
			n = self.actions[sa]
			if len(n.comment) > 0 :
				f.write ("// " + n.comment + "\n")
			f.write("// %s: %s\n" % (n.node_type, n.name))
			if "ActionID" in n.attributes.keys() :
				attr = n.attributes["ActionID"]
				f.write("//\t%s=%s" % (attr[0], attr[1]))
				if attr[2] != "" :
					f.write("\t// %s" % attr[2])
				f.write("\n")
				f.write("RACTION %s=%s\n" % (n.name, attr[1]))
			f.write("\n")

		f.write ("// *** Output completed. ***\n")

		f.close()
		ObjRenderer.end_render(self)

	def render_node(self, node, file) :
		elemlist = []
		type = ""
		if node.node_type == "Drive" :
			elemlist = [("Priority",1), ("Interval",0), ("Sense",0), ("Comparator",2),
				("SenseValue",1), ("SenseHysteresis",1), ("SenseFlexLatchHysteresis",0),
				("RampIncrement",0), ("UrgencyMultiplier",0), ("RampInterval",0)]
			type="D"
		elif node.node_type == "Competence" :
			elemlist = [("UseORWithinCEGroup",0)]
			type="C"
		elif node.node_type == "Action" :
			elemlist = [("Action",0), ("ActionValue",1)]
			type="A"
		elif node.node_type == "CompetenceElement" :
			elemlist = [("Priority",1), ("RetryLimit",0), ("Sense",0), ("Comparator",2),
				("SenseValue",1), ("SenseHysteresis",1), ("SenseFlexLatchHysteresis",0)]
			type="E"
		elif node.node_type == "ActionPattern" :
			elemlist = []
			type="P"
		elif node.node_type == "ActionPatternElement" :
			elemlist = [("Order",1)]
			type="L"

		# now write out supplied params as comments in correct order
		for elem in elemlist :
			if elem[0] in node.attributes.keys() :
				attr = node.attributes[elem[0]]
				# print attr
				file.write("//\t%s=%s" % (attr[0], attr[1]))
				if attr[2] != "" :
					file.write("\t// %s" % attr[2])
				file.write("\n")
		# now write out the actual planner command line
		# use defaults from elemlist if params not supplied
		file.write("PLAN A %s %d" % (type, node.node_id))
		if node.node_type in ("CompetenceElement", "ActionPatternElement") :
			parID = 0
			childID = 0
			if len(node.parents) == 1 :
				parID = self.nodes[node.parents[0]].node_id
			if len(node.children) == 1 :
				childID = self.nodes[node.children[0]].node_id
			file.write(" %d %d" % (parID, childID))
		elif node.node_type in ("Drive") :
			childID = 0
			if len(node.children) == 1 :
				childID = self.nodes[node.children[0]].node_id
			file.write(" %d" % childID)
		
		for elem in elemlist :
			val = elem[1]
			if elem[0] in node.attributes.keys() :
				str = node.attributes[elem[0]][1]
				if elem[0] == "Sense" :
					val = self.get_sense_id(str)
				elif elem[0] == "Action" :
					val = self.get_action_id(str)
				elif elem[0] == "Comparator" :
					val = self.get_comparator_val(str)
				else :
					val = str
			# print val
			file.write(" %s" % val)
		file.write("\n")
			
	def get_sense_id(self, sense_name) :
		# print "sense_name ", sense_name
		sense_id = 0
		if sense_name in self.senses.keys() :
			if "SenseID" in self.senses[sense_name].attributes.keys() :
				sense_id = self.senses[sense_name].attributes["SenseID"][1]
		return sense_id

	def get_action_id(self, action_name) :
		# print "action_name ", action_name
		action_id = 0
		if action_name in self.actions.keys() :
			if "ActionID" in self.actions[action_name].attributes.keys() :
				action_id = self.actions[action_name].attributes["ActionID"][1]
		return action_id

	def get_comparator_val(self, comparator_name) :
		# print "comparator_name ", comparator_name
		comp_val = 0
		comp_values = {'EQ':0, 'NE':1, 'GT':2, 'LT':3, 'TR':4, 'FL':5,}
		if comparator_name in comp_values.keys() :
			comp_val = comp_values[comparator_name]
		return comp_val		
			

# dia-python keeps a reference to the renderer class and uses it on demand
dia.register_export ("Instinct Plan Generation", "inst", InstinctRenderer())
