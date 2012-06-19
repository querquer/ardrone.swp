
(cl:in-package :asdf)

(defsystem "follow_line-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LinePos" :depends-on ("_package_LinePos"))
    (:file "_package_LinePos" :depends-on ("_package"))
  ))