#!/usr/bin/env python

# Copyright (c) 2011, Dorian Scholz, TU Darmstadt
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the TU Darmstadt nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

from __future__ import division
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, Signal, Slot
from python_qt_binding.QtGui import QIcon 
from python_qt_binding.QtWidgets import QHeaderView, QMenu, QTreeWidgetItem, QWidget, QSlider, QAbstractItemView


import roslib
import rospkg
import rospy
from rospy.exceptions import ROSException

from .topic_info import TopicInfo

import numpy as np


class TopicWidget(QWidget):
	"""
	main class inherits from the ui window class.

	You can specify the topics that the topic pane.

	TopicWidget.start must be called in order to update topic pane.
	"""

	SELECT_BY_NAME = 0
	SELECT_BY_MSGTYPE = 1

	# _column_names = ['topic', 'type', 'bandwidth', 'rate', 'value', 'checkbox']
	_column_names = ['topic', 'type', 'value', 'checkbox']

	selectionChanged = Signal(list, name='selectionChanged')
	topicsRefreshed = Signal(name='topicsRefreshed')


	def __init__(self, plugin=None, selected_topics=None, select_topic_type=SELECT_BY_NAME):
		"""
		@type selected_topics: list of tuples.
		@param selected_topics: [($NAME_TOPIC$, $TYPE_TOPIC$), ...]
		@type select_topic_type: int
		@param select_topic_type: Can specify either the name of topics or by
								  the type of topic, to filter the topics to
								  show. If 'select_topic_type' argument is
								  None, this arg shouldn't be meaningful.
		"""
		super(TopicWidget, self).__init__()

		self._select_topic_type = select_topic_type

		rp = rospkg.RosPack()
		ui_file = os.path.join(rp.get_path('rqt_topic'), 'resource', 'TopicWidget.ui')
		loadUi(ui_file, self)
		self._plugin = plugin
		self.topics_tree_widget.sortByColumn(0, Qt.AscendingOrder)
		header = self.topics_tree_widget.header()
		try:
			setSectionResizeMode = header.setSectionResizeMode  # Qt5
		except AttributeError:
			setSectionResizeMode = header.setResizeMode  # Qt4
		for i in range(len(self._column_names)):
			setSectionResizeMode(i, QHeaderView.Stretch)

		header.setStretchLastSection(False)
		setSectionResizeMode(0, QHeaderView.Stretch)
		setSectionResizeMode(3, QHeaderView.ResizeToContents)

		self.topics_tree_widget.setSelectionMode(QAbstractItemView.MultiSelection)
		
		# setSectionResizeMode(3, QHeaderView.Fixed)

		# header.customContextMenuRequested.connect(self.handle_header_view_customContextMenuRequested)
		# header.setContextMenuPolicy(Qt.CustomContextMenu)

		# Whether to get all topics or only the topics that are set in advance.
		# Can be also set by the setter method "set_selected_topics".
		self._selected_topics = selected_topics

		self._selected_items = []

		self._current_topic_list = []
		self._topics = {}
		self._tree_items = {}
		self._column_index = {}

		self.values = {}

		for column_name in self._column_names:
			self._column_index[column_name] = len(self._column_index)

		self.topics_tree_widget.itemExpanded.connect(self.expanded)
		self.topics_tree_widget.itemCollapsed.connect(self.collapsed)

		# self.refresh_topics()

		# init and start update timer
		self._timer_refresh_topics = QTimer(self)
		self._timer_refresh_topics.timeout.connect(self.refresh_topics)

		self.topics_tree_widget.setAlternatingRowColors(True)

		self.topics_tree_widget.header().resizeSection(3, 50)

		self.topics_tree_widget.itemSelectionChanged.connect(self.selection_changed)



	def set_topic_specifier(self, specifier):
		self._select_topic_type = specifier

	def start(self):
		"""
		This method needs to be called to start updating topic pane.
		"""
		self._timer_refresh_topics.start(1000)


	@Slot()
	def selection_changed(self):

		selected = self.get_selected()

		for key, item in self._tree_items.items():
			if item.isSelected():
				item.setText(3, '{%d}' %  selected.index(key))
			else:
				item.setText(3, '')

		self.selectionChanged.emit(selected)

		
		

		# for item in selected:
		#     # item.setData(self._column_names.index("checkbox") ,True, Qt.CheckStateRole)
		#     item.setCheckState(self._column_names.index("checkbox"), Qt.Checked)

		# for _, item in self._tree_items.items():
		#     if item not in selected:
		#         # item.setData(self._column_names.index("checkbox") ,False, Qt.CheckStateRole)                
		#         item.setCheckState(self._column_names.index("checkbox"), Qt.Unchecked)

		# self._selected_items = map(lambda item: item.data(0, Qt.EditRole), selected)

		pass

	@Slot('QTreeWidgetItem')
	def expanded(self, item):

		name = item.data(0, Qt.UserRole)

		if not isinstance(item, TreeWidgetItem) or not item._is_topic:
			return

		print "expanded", name
		self._topics[item._topic_name]['info'].start_monitoring()


	@Slot('QTreeWidgetItem')
	def collapsed(self, item):

		name = item.data(0, Qt.UserRole)

		if not isinstance(item, TreeWidgetItem) or not item._is_topic:
			return

		print "collapsed", name
		self._topics[item._topic_name]['info'].stop_monitoring()

	def unselect(self, items):
		if not hasattr(items, "__iter__"):
			items = [items]
		for item in items:
			if item in self._selected_items:
				self._selected_items.remove(item)
				self._tree_items[item].setData(self._column_names.index("checkbox"), Qt.CheckStateRole, False)


	def clear_selection(self):
		self.topics_tree_widget.clearSelection()

		items = self.get_selected()
		for item in items:
			self._tree_items[item].setData(self._column_names.index("checkbox"), Qt.CheckStateRole, False)
		self._selected_items = []

	def get_selected(self):
		# param_name:desc
		# print self._selected_items
		self._selected_items = [ item._topic_name for item in self.topics_tree_widget.selectedItems() ]

		# print self._selected_items
		# print self.topics_tree_widget.selectedItems()

		return self._selected_items

	def get_selected_values(self):
		# param_name:desc
		self.get_selected()
		items = self._selected_items       
		values = {item: self.values[item] for item in items if item in self.values}

		# print "NEW_VALUES: ", values        

		return values

	def get_selected_types(self):
		# self._tree_items[topic_name].setText(self._column_index['value'], repr(message))
		self.get_selected()
		f = lambda item: self._tree_items[item].data(self._column_index['type'], Qt.EditRole)
		selected_types = map(f, self._selected_items)
		# print selected_types
		return selected_types


	@Slot()
	def refresh_topics(self):
		"""
		refresh tree view items
		"""
		# print "is refreshing"
		


		try:
			if self._selected_topics is None:
				topic_list = rospy.get_published_topics()
				if topic_list is None:
					rospy.logerr('Not even a single published topic found. Check network configuration')
					return
			else:  # Topics to show are specified.
				topic_list = self._selected_topics
				topic_specifiers_server_all = None
				topic_specifiers_required = None

				rospy.logdebug('refresh_topics) self._selected_topics=%s' % (topic_list,))

				if self._select_topic_type == self.SELECT_BY_NAME:
					topic_specifiers_server_all = [name for name, type in rospy.get_published_topics()]
					topic_specifiers_required = [name for name, type in topic_list]
				elif self._select_topic_type == self.SELECT_BY_MSGTYPE:
					# The topics that are required (by whoever uses this class).
					topic_specifiers_required = [type for name, type in topic_list]

					# The required topics that match with published topics.
					topics_match = [(name, type) for name, type in rospy.get_published_topics() if type in topic_specifiers_required]
					topic_list = topics_match
					rospy.logdebug('selected & published topic types=%s' % (topic_list,))

				rospy.logdebug('server_all=%s\nrequired=%s\ntlist=%s' % (topic_specifiers_server_all, topic_specifiers_required, topic_list))
				if len(topic_list) == 0:
					rospy.logerr('None of the following required topics are found.\n(NAME, TYPE): %s' % (self._selected_topics,))
					return
		except IOError as e:
			rospy.logerr("Communication with rosmaster failed: {0}".format(e.strerror))
			return

		if self._current_topic_list != topic_list:
			self._current_topic_list = topic_list



			# start new topic dict
			new_topics = {}

			for topic_name, topic_type in topic_list:
				# if topic is new or has changed its type
				if topic_name not in self._topics or \
				   self._topics[topic_name]['type'] != topic_type:
					# create new TopicInfo
					topic_info = TopicInfo(topic_name, topic_type)
					message_instance = None
					if topic_info.message_class is not None:
						message_instance = topic_info.message_class()
					# add it to the dict and tree view
					topic_item = self._recursive_create_widget_items(self.topics_tree_widget, topic_name, topic_type, message_instance)
					new_topics[topic_name] = {
					   'item': topic_item,
					   'info': topic_info,
					   'type': topic_type,
					}
				else:
					# if topic has been seen before, copy it to new dict and
					# remove it from the old one
					new_topics[topic_name] = self._topics[topic_name]
					del self._topics[topic_name]

			# clean up old topics
			for topic_name in self._topics.keys():
				self._topics[topic_name]['info'].stop_monitoring()
				index = self.topics_tree_widget.indexOfTopLevelItem(
										   self._topics[topic_name]['item'])
				self.topics_tree_widget.takeTopLevelItem(index)
				del self._topics[topic_name]

			# switch to new topic dict
			self._topics = new_topics

			self.selectionChanged.emit(self.get_selected())

		self._update_topics_data()

		self.topicsRefreshed.emit()

	def remove(self, key):
		item = self._tree_items[key]
		index = self.topics_tree_widget.indexOfTopLevelItem(item)
		self.topics_tree_widget.takeTopLevelItem(index)
		del self._tree_items[key]

	def _update_topics_data(self):

		selected_dict = { self._selected_items[index] : index for index in range(len(self._selected_items)) }
		# print selected_dict


		for topic in self._topics.values():
			topic_info = topic['info']
			if topic_info.monitoring:
				# update rate
				rate, _, _, _ = topic_info.get_hz()
				rate_text = '%1.2f' % rate if rate != None else 'unknown'

				# update bandwidth
				bytes_per_s, _, _, _ = topic_info.get_bw()
				if bytes_per_s is None:
					bandwidth_text = 'unknown'
				elif bytes_per_s < 1000:
					bandwidth_text = '%.2fB/s' % bytes_per_s
				elif bytes_per_s < 1000000:
					bandwidth_text = '%.2fKB/s' % (bytes_per_s / 1000.)
				else:
					bandwidth_text = '%.2fMB/s' % (bytes_per_s / 1000000.)

				# update values
				value_text = ''
				self.update_value(topic_info._topic_name, topic_info.last_message)

			else:
				rate_text = ''
				bandwidth_text = ''
				value_text = 'not monitored' if topic_info.error is None else topic_info.error

			# self._tree_items[topic_info._topic_name].setText(self._column_index['rate'], rate_text)
			# self._tree_items[topic_info._topic_name].setText(self._column_index['bandwidth'], bandwidth_text)
			self._tree_items[topic_info._topic_name].setText(self._column_index['value'], value_text)

	def update_value(self, topic_name, message, vtype = None):

		value = message
		if hasattr(value, '__iter__'):
			try:
				value = np.array(value)
				pass
			except:
				pass
		
		self.values.update({topic_name:value})

		if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
			slots = zip(message.__slots__, message._slot_types)
			for slot_name, slot_type in slots:
				self.update_value(topic_name + '/' + slot_name, getattr(message, slot_name), vtype = slot_type)

		elif type(message) in (list, tuple) and (len(message) > 0):# and hasattr(message[0], '__slots__'):

			for index, slot in enumerate(message):
				if topic_name + '[%d]' % index in self._tree_items:
					self.update_value(topic_name + '[%d]' % index, slot)
				else:
					base_type_str, _ = self._extract_array_info(self._tree_items[topic_name].text(self._column_index['type']))
					self._recursive_create_widget_items(self._tree_items[topic_name], topic_name + '[%d]' % index, base_type_str, slot)
			# remove obsolete children
			if len(message) < self._tree_items[topic_name].childCount():
				for i in range(len(message), self._tree_items[topic_name].childCount()):
					item_topic_name = topic_name + '[%d]' % i
					self._recursive_delete_widget_items(self._tree_items[item_topic_name])
		else:
			if topic_name in self._tree_items:
				self._tree_items[topic_name].setText(self._column_index['value'], repr(message))

	def _extract_array_info(self, type_str):
		array_size = None
		if '[' in type_str and type_str[-1] == ']':
			type_str, array_size_str = type_str.split('[', 1)
			array_size_str = array_size_str[:-1]
			if len(array_size_str) > 0:
				array_size = int(array_size_str)
			else:
				array_size = 0

		return type_str, array_size

	def _recursive_create_widget_items(self, parent, topic_name, type_name, message):
		if parent is self.topics_tree_widget:
			# show full topic name with preceding namespace on toplevel item
			topic_text = topic_name
			# item = TreeWidgetItem(self._toggle_monitoring, topic_name, parent)


			_item = parent
			topic_names = topic_name.split('/')
			name_space = ""
			for name in topic_names:
				if name == "":
					continue
				_name = "/" + name
				name_space = name_space + _name
				if name_space not in self._tree_items:
					print ">>>"+name_space

					is_topic = False

					if name_space == topic_name:
						is_topic = True


					_item = TreeWidgetItem(self._toggle_monitoring, name_space, _item, is_topic = is_topic)
					_item.setText(self._column_index['topic'], _name)
					_item.setText(self._column_index['type'], type_name)
					_item.setData(0, Qt.UserRole, name_space)

					self._tree_items[name_space] = _item




				_item = self._tree_items[name_space]

			item = _item



		else:
			topic_text = topic_name.split('/')[-1]
			if '[' in topic_text:
				topic_text = topic_text[topic_text.index('['):]


			item = TreeWidgetItem(self._toggle_monitoring, topic_name, parent)
			# item = QTreeWidgetItem(parent)
			# item.setCheckState(0, Qt.Unchecked)

			item.setText(self._column_index['topic'], topic_text)
			item.setText(self._column_index['type'], type_name)
			item.setData(0, Qt.UserRole, topic_name)
			self._tree_items[topic_name] = item


		
		if hasattr(message, '__slots__') and hasattr(message, '_slot_types'):
			for slot_name, type_name in zip(message.__slots__, message._slot_types):
				self._recursive_create_widget_items(item, topic_name + '/' + slot_name, type_name, getattr(message, slot_name))

		else:
			base_type_str, array_size = self._extract_array_info(type_name)
			try:
				base_instance = roslib.message.get_message_class(base_type_str)()
			except (ValueError, TypeError):
				base_instance = None
			if array_size is not None and hasattr(base_instance, '__slots__'):
				for index in range(array_size):
					self._recursive_create_widget_items(item, topic_name + '[%d]' % index, base_type_str, base_instance)
		return item



	def _toggle_monitoring(self, topic_name):
		# item = self._tree_items[topic_name]
		# if item.checkState(3):
		#     print "start %s" % topic_name
		#     if topic_name not in self._selected_items:
		#         self._selected_items.append(topic_name)
		#         # item.setText(3, str(self._selected_items.index(topic_name)))
		#     # self._topics[topic_name]['info'].start_monitoring()
		# else:
		#     print "stop %s" % topic_name
		#     if topic_name in self._selected_items:
		#         self._selected_items.remove(topic_name)
		#     item.setText(3,'')

		self.get_selected()

		sel = self._selected_items
		for i in range(len(sel)):
			_item = self._tree_items[sel[i]]
			_item.setText(3, '{%d}' % i )

			# self._topics[topic_name]['info'].stop_monitoring()

		self.selectionChanged.emit(self.get_selected())
		

	def _recursive_delete_widget_items(self, item):
		def _recursive_remove_items_from_tree(item):
			for index in reversed(range(item.childCount())):
				_recursive_remove_items_from_tree(item.child(index))
			topic_name = item.data(0, Qt.UserRole)
			del self._tree_items[topic_name]
		_recursive_remove_items_from_tree(item)
		item.parent().removeChild(item)

	@Slot('QPoint')
	def handle_header_view_customContextMenuRequested(self, pos):
		header = self.topics_tree_widget.header()

		# show context menu
		menu = QMenu(self)
		action_toggle_auto_resize = menu.addAction('Toggle Auto-Resize')
		action = menu.exec_(header.mapToGlobal(pos))

		# evaluate user action
		if action is action_toggle_auto_resize:
			if header.resizeMode(0) == QHeaderView.ResizeToContents:
				header.setResizeMode(QHeaderView.Interactive)
			else:
				header.setResizeMode(QHeaderView.ResizeToContents)

	@Slot('QPoint')
	def on_topics_tree_widget_customContextMenuRequested(self, pos):
		item = self.topics_tree_widget.itemAt(pos)
		if item is None:
			return

		# show context menu
		menu = QMenu(self)
		action_item_expand = menu.addAction(QIcon.fromTheme('zoom-in'), 'Expand All Children')
		action_item_collapse = menu.addAction(QIcon.fromTheme('zoom-out'), 'Collapse All Children')
		action = menu.exec_(self.topics_tree_widget.mapToGlobal(pos))

		# evaluate user action
		if action in (action_item_expand, action_item_collapse):
			expanded = (action is action_item_expand)

			def recursive_set_expanded(item):
				item.setExpanded(expanded)
				for index in range(item.childCount()):
					recursive_set_expanded(item.child(index))
			recursive_set_expanded(item)

	def shutdown_plugin(self):
		for topic in self._topics.values():
			topic['info'].stop_monitoring()
		self._timer_refresh_topics.stop()

	def set_selected_topics(self, selected_topics):
		"""
		@param selected_topics: list of tuple. [(topic_name, topic_type)]
		@type selected_topics: []
		"""
		rospy.logdebug('set_selected_topics topics={}'.format(
														 len(selected_topics)))
		self._selected_topics = selected_topics

	# TODO(Enhancement) Save/Restore tree expansion state
	def save_settings(self, plugin_settings, instance_settings):
		header_state = self.topics_tree_widget.header().saveState()
		instance_settings.set_value('tree_widget_header_state', header_state)

	def restore_settings(self, pluggin_settings, instance_settings):
		if instance_settings.contains('tree_widget_header_state'):
			header_state = instance_settings.value('tree_widget_header_state')
			if not self.topics_tree_widget.header().restoreState(header_state):
				rospy.logwarn("rqt_topic: Failed to restore header state.")

class TreeWidgetItem(QTreeWidgetItem):

	def __init__(self, check_state_changed_callback, topic_name, parent=None, is_topic = False):
		super(TreeWidgetItem, self).__init__(parent)
		self._check_state_changed_callback = check_state_changed_callback
		self._topic_name = topic_name
		# self.setCheckState(3, Qt.Unchecked)
		self._is_topic = is_topic
		
		# self._slider = QSlider(Qt.Horizontal)

		# tree = self.treeWidget()
		# tree.setItemWidget(self, 4, self._slider)        

		# self._slider.valueChanged.connect(self.sliderValueChanged)

		# print ">>>>>>>>>>>>>> %s" %  self._slider

	# @Slot(int)
	# def sliderValueChanged(self, value):
	#     self.setText(2, str(value))
	#     print "value changed to %d" % value
	#     pass



		# print(topic_name)

	def setData(self, column, role, value):
		# if role == Qt.CheckStateRole:
		#     state = self.checkState(column)
			# if state != self.isSelected():
			#     self.setSelected(state)
			#     return

		
		# if state:
		#     super(TreeWidgetItem, self).setData(column, Qt.UserRole, self._selected_items.index(self._topic_name))    

		super(TreeWidgetItem, self).setData(column, role, value)



		# if role == Qt.CheckStateRole and state != self.checkState(column):
		# self._check_state_changed_callback(self._topic_name)



