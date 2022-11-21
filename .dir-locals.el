;;; .dir-locals.el --

;;; Commentary:

;;; Code:
((c++-mode .
      ((eval  . (progn
                  (let ((includes-list (list
                                        (substitute-in-file-name "$rcsw/include")
                                        (substitute-in-file-name "$rcppsw/include")
                                        (substitute-in-file-name "/opt/ros/noetic/include")
                                        (substitute-in-file-name "$localroot/ros/include")
                                        (concat (projectile-project-root)
                                                "include")
                                        "/usr/include/lua5.2"
                                        "/usr/include/x86_64-linux-gnu/qt5/"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtWidgets"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtGui"
                                        "/usr/include/x86_64-linux-gnu/qt5/QtCore"
                                        "/usr/lib/x86_64-linux-gnu/qt5//mkspecs/linux-g++-64"
                                        )))
                    (setq company-c-headers-path-system
                          (list
                           (substitute-in-file-name
                            "$localroot/ros/include")
                           (substitute-in-file-name "$localroot/system/include")
                           )
                          )
                    (setq company-c-headers-path-user
                          (list
                           (substitute-in-file-name
                            "$localroot/include")
                           ))
                    (setq flycheck-clang-include-path includes-list)
                    (add-to-list 'flycheck-clang-args "-fPIC")
                    (add-to-list 'flycheck-clang-args "-std=c++17")
                    (add-to-list 'flycheck-clang-args "-Wno-pragma-once-outside-header")
                    (add-to-list 'flycheck-clang-definitions
                                 "COSM_ENABLE_PAL_TARGET_ARGOS")
                    ;; (add-to-list 'flycheck-clang-definitions
                    ;;              "COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_FOOTBOT")
                    (add-to-list 'flycheck-clang-definitions
                                 "COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_DRONE")
                    (add-to-list 'flycheck-clang-definitions
                                 "COSM_HAL_TARGET_HAS_QUADROTOR")
                    (add-to-list 'flycheck-clang-definitions
                                 "COSM_HAL_TARGET_HAS_2D_BINDINGS")
                    (add-to-list 'flycheck-clang-definitions
                                 "COSM_HAL_TARGET_HAS_3D_BINDINGS")

                    (add-to-list 'flycheck-clang-definitions
                                 "LIBRA_ER=LIBRA_ER_ALL")
                    (add-to-list 'flycheck-clang-definitions
                                 "RCPPSW_ER_SYSTEM_LOG4CXX")
                    (add-to-list 'flycheck-clang-args
                                 (concat "-isystem" (substitute-in-file-name
                                                     "$localroot/system/include")))


                    (setq flycheck-gcc-include-path includes-list)
                    (add-to-list 'flycheck-gcc-args "-fPIC")
                    (add-to-list 'flycheck-gcc-args "-std=c++17")
                    (add-to-list 'flycheck-gcc-definitions
                                 "COSM_ENABLE_PAL_TARGET_ARGOS")
                    (add-to-list 'flycheck-gcc-definitions
                                 "COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_DRONE")
                    (add-to-list 'flycheck-gcc-definitions
                                 "COSM_HAL_TARGET_HAS_QUADROTOR")
                    (add-to-list 'flycheck-gcc-definitions
                                 "COSM_HAL_TARGET_HAS_2D_BINDINGS")
                    (add-to-list 'flycheck-gcc-definitions
                                 "COSM_HAL_TARGET_HAS_3D_BINDINGS")
                    ;; (add-to-list 'flycheck-gcc-definitions
                    ;;              "COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_FOOTBOT")
                    (add-to-list 'flycheck-gcc-definitions
                                 "RCPPSW_ER_SYSTEM_LOG4CXX")
                    (add-to-list 'flycheck-gcc-definitions
                                 "LIBRA_ER=LIBRA_ER_ALL")

                    (add-to-list 'flycheck-gcc-args
                                 (concat "-isystem" (substitute-in-file-name
                                                     "$localroot/system/include")))

                    (let ((cc-search-dirs (list (concat (projectile-project-root) "include/cosm/*/*")
                                                (concat (projectile-project-root) "src/*/*")
                                                (concat (projectile-project-root) "include"))
                                          ))
                      (setq cc-search-directories cc-search-dirs))
                    )
              )
         ))
      )
 (nil . ((eval .
               (progn
                 (add-to-list 'projectile-globally-ignored-directories
                              "docs/doxyoutput")
                 )
               ))
      )
 
 )

;;; end of .dir-locals.el
